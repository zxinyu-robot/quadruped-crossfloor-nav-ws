// DDS 同机通信诊断工具（吞吐 / 时延 / 反压 / QoS 交互）。
//
// 设计目标：把「整体耗时」换成可归因的指标——
//   写端：单次 dds_write 延迟分布、成功/失败计数、失败返回码分布、生效 QoS
//   读端：端到端时延分布、首尾窗口对比、实际接收数
//
// 用法示例：
//   dds_bench --role sub --count 2000 --size 65536 --depth 5 --reliability reliable
//   dds_bench --role pub --count 2000 --size 65536 --depth 5 --reliability reliable
//   dds_bench --role sub --duration-s 1200 --size 65536 --rate-hz 10 --bucket-s 60   # soak
//   dds_bench --role sub --count 2000 --consumer-delay-ms 5    # 模拟慢消费
#include <dds/dds.h>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <string>
#include <thread>
#include <vector>

#include "motionslam_msgs/msg/RuntimeEnvelope.h"

namespace {

long long now_ns() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

struct Options {
    std::string role = "pub";
    std::string topic = "rt/bench/envelope";
    int count = -1;
    int size = 65536;
    double duration_s = 0.0;
    double rate_hz = 0.0;  // 0 = 不限速
    int bucket_s = 60;
    std::string reliability = "reliable";
    std::string durability = "volatile";
    int depth = 5;
    int max_blocking_ms = 200;
    int await_match_ms = 8000;
    int idle_ms = 2000;            // 收到首包后的空闲窗口（墙钟）
    int first_packet_ms = 10000;   // 首包最长等待；超时且无数据 → NO_DATA
    int overall_ms = 60000;        // 单次接收总上限
    int delay_after_match_ms = 0;  // 发布端：匹配后再等多久才发（T1）
    int pause_after = -1;          // 发布端：发到该序号后暂停一次（T4）
    int pause_ms = 0;
    int consumer_delay_ms = 0;  // 取完之后的人为处理耗时（模拟处理能力）
    int take_interval_ms = 0;   // 每次等待前的强制间隔（模拟取数频率不足，造成 RHC 积压）
    bool default_qos = false;    // 诊断用：用 DDS 默认 QoS，跳过全部 qset
    bool topic_qos = true;       // 诊断用：是否把自定义 QoS 传给 dds_create_topic
};

dds_qos_t* make_qos(const Options& opt) {
    if (opt.default_qos) return nullptr;
    dds_qos_t* qos = dds_create_qos();
    const bool be = opt.reliability == "best_effort";
    dds_qset_reliability(qos, be ? DDS_RELIABILITY_BEST_EFFORT : DDS_RELIABILITY_RELIABLE,
                         DDS_MSECS(opt.max_blocking_ms));
    dds_qset_durability(qos, opt.durability == "transient_local" ? DDS_DURABILITY_TRANSIENT_LOCAL
                                                                 : DDS_DURABILITY_VOLATILE);
    // 深度只决定本地保留的历史样本数，不参与 requested/offered 匹配。
    dds_qset_history(qos, DDS_HISTORY_KEEP_LAST, static_cast<int32_t>(opt.depth));
    return qos;
}

int await_match(dds_entity_t entity, int timeout_ms) {
    for (int i = 0; i < timeout_ms / 50; ++i) {
        dds_publication_matched_status_t pub_st;
        if (dds_get_publication_matched_status(entity, &pub_st) == DDS_RETCODE_OK &&
            pub_st.current_count > 0) {
            return static_cast<int>(pub_st.current_count);
        }
        dds_subscription_matched_status_t sub_st;
        if (dds_get_subscription_matched_status(entity, &sub_st) == DDS_RETCODE_OK &&
            sub_st.current_count > 0) {
            return static_cast<int>(sub_st.current_count);
        }
        dds_sleepfor(DDS_MSECS(50));
    }
    return 0;
}

double pct(std::vector<double> v, double p) {
    if (v.empty()) return 0.0;
    const size_t idx = static_cast<size_t>(p * static_cast<double>(v.size() - 1));
    std::nth_element(v.begin(), v.begin() + static_cast<long>(idx), v.end());
    return v[idx];
}

double max_of(const std::vector<double>& v) { return v.empty() ? 0.0 : *std::max_element(v.begin(), v.end()); }

void print_qos(const Options& opt) {
    std::printf("\"qos\": {\"reliability\": \"%s\", \"durability\": \"%s\", \"depth\": %d, "
                "\"history\": \"keep_last\", \"max_blocking_ms\": %d, "
                "\"resource_limits\": \"unlimited\"}",
                opt.reliability.c_str(), opt.durability.c_str(), opt.depth, opt.max_blocking_ms);
}

int run_pub(const Options& opt, dds_entity_t pp, dds_qos_t* qos) {
    // 状态机：INIT → READY → DISCOVERING → MATCHED → SENDING → DONE / INVALID
    // READY 只表示端点已创建、已开始监听发现，不代表已匹配。
    std::printf("{\"role\": \"pub\", \"stage\": \"INIT\"}\n");
    std::fflush(stdout);

    dds_entity_t topic = dds_create_topic(pp, &motionslam_msgs_msg_dds__RuntimeEnvelope__desc,
                                          opt.topic.c_str(), opt.topic_qos ? qos : nullptr, nullptr);
    if (topic < 0) return 1;
    dds_entity_t writer = dds_create_writer(pp, topic, qos, nullptr);
    if (writer < 0) return 1;

    std::printf("{\"role\": \"pub\", \"stage\": \"READY\"}\n");
    std::printf("{\"role\": \"pub\", \"stage\": \"DISCOVERING\"}\n");
    std::fflush(stdout);

    const int matched = await_match(writer, opt.await_match_ms);
    // Match Gate：没有匹配到的运行不得产出任何性能数字。
    if (matched < 1) {
        std::printf("{\"role\": \"pub\", \"stage\": \"INVALID\", \"valid\": false, "
                    "\"reason\": \"match_timeout\", \"matched\": 0}\n");
        return 3;
    }
    std::printf("{\"role\": \"pub\", \"stage\": \"MATCHED\", \"matched\": %d}\n", matched);
    if (opt.delay_after_match_ms > 0) dds_sleepfor(DDS_MSECS(opt.delay_after_match_ms));
    std::printf("{\"role\": \"pub\", \"stage\": \"SENDING\"}\n");
    std::fflush(stdout);
    const long long t0 = now_ns();
    const long long deadline =
        opt.duration_s > 0 ? t0 + static_cast<long long>(opt.duration_s * 1e9) : 0;
    const long long interval = opt.rate_hz > 0 ? static_cast<long long>(1e9 / opt.rate_hz) : 0;

    int attempted = 0;
    int write_ok = 0;
    std::vector<double> write_lat_us;
    std::map<int, int> rc_hist;
    long long next_at = t0;
    long long blocked_ns = 0;

    while (true) {
        if (opt.count >= 0 && attempted >= opt.count) break;
        if (deadline > 0 && now_ns() > deadline) break;

        motionslam_msgs_msg_dds__RuntimeEnvelope_ msg;
        std::memset(&msg, 0, sizeof msg);
        msg.schema_version = 1;
        msg.channel_id = static_cast<uint32_t>(attempted);  // 序列号，供接收端算真实丢包
        msg.stamp_ns = now_ns();
        msg.payload._length = static_cast<uint32_t>(opt.size);
        msg.payload._buffer = static_cast<uint8_t*>(dds_alloc(static_cast<size_t>(opt.size)));
        std::memset(msg.payload._buffer, static_cast<int>(attempted & 0xff),
                    static_cast<size_t>(opt.size));
        msg.payload._release = true;

        const long long w0 = now_ns();
        const dds_return_t rc = dds_write(writer, &msg);
        const long long w1 = now_ns();

        const double lat_us = static_cast<double>(w1 - w0) / 1000.0;
        write_lat_us.push_back(lat_us);
        if (lat_us > 1000.0) blocked_ns += (w1 - w0);  // 超过 1 ms 视为一次阻塞
        ++attempted;
        if (rc == DDS_RETCODE_OK) {
            ++write_ok;
        } else {
            ++rc_hist[static_cast<int>(rc)];
        }
        dds_free(msg.payload._buffer);

        if (opt.pause_after >= 0 && attempted == opt.pause_after && opt.pause_ms > 0) {
            std::printf("{\"role\": \"pub\", \"stage\": \"PAUSED\", \"at\": %d}\n", attempted);
            std::fflush(stdout);
            dds_sleepfor(DDS_MSECS(opt.pause_ms));
        }
        if (interval > 0) {
            next_at += interval;
            const long long sleep_ns = next_at - now_ns();
            if (sleep_ns > 0) std::this_thread::sleep_for(std::chrono::nanoseconds(sleep_ns));
        }
    }
    const long long t1 = now_ns();
    dds_write_flush(writer);
    dds_sleepfor(DDS_MSECS(800));

    const double secs = static_cast<double>(t1 - t0) / 1e9;
    const double mb = static_cast<double>(write_ok) * opt.size / (1024.0 * 1024.0);

    std::printf("{\"role\": \"pub\", \"stage\": \"DONE\", \"valid\": true, \"matched\": %d, "
                "\"attempted\": %d, \"write_ok\": %d, "
                "\"write_fail\": %d, \"write_fail_rc\": {",
                matched, attempted, write_ok, attempted - write_ok);
    bool first = true;
    for (const auto& kv : rc_hist) {
        std::printf("%s\"%d\": %d", first ? "" : ", ", kv.first, kv.second);
        first = false;
    }
    std::printf("}, \"size\": %d, \"seconds\": %.3f, \"msg_per_s\": %.0f, \"mb_per_s\": %.1f, "
                "\"write_latency_us\": {\"p50\": %.1f, \"p95\": %.1f, \"p99\": %.1f, \"max\": %.1f}, "
                "\"write_blocked_total_ms\": %.1f, ",
                opt.size, secs, static_cast<double>(write_ok) / (secs > 0 ? secs : 1),
                mb / (secs > 0 ? secs : 1), pct(write_lat_us, 0.50), pct(write_lat_us, 0.95),
                pct(write_lat_us, 0.99), max_of(write_lat_us),
                static_cast<double>(blocked_ns) / 1e6);
    print_qos(opt);
    std::printf("}\n");
    return attempted - write_ok == 0 ? 0 : 1;
}

int run_sub(const Options& opt, dds_entity_t pp, dds_qos_t* qos) {
    std::printf("{\"role\": \"sub\", \"stage\": \"INIT\"}\n");
    std::fflush(stdout);

    dds_entity_t topic = dds_create_topic(pp, &motionslam_msgs_msg_dds__RuntimeEnvelope__desc,
                                          opt.topic.c_str(), opt.topic_qos ? qos : nullptr, nullptr);
    if (topic < 0) return 1;
    dds_entity_t reader = dds_create_reader(pp, topic, qos, nullptr);
    if (reader < 0) return 1;

    std::printf("{\"role\": \"sub\", \"stage\": \"READY\"}\n");
    std::printf("{\"role\": \"sub\", \"stage\": \"DISCOVERING\"}\n");
    std::fflush(stdout);

    const int matched = await_match(reader, opt.await_match_ms);
    if (matched < 1) {
        std::printf("{\"role\": \"sub\", \"stage\": \"INVALID\", \"valid\": false, "
                    "\"reason\": \"match_timeout\", \"matched\": 0}\n");
        return 3;
    }
    std::printf("{\"role\": \"sub\", \"stage\": \"MATCHED\", \"matched\": %d}\n", matched);
    std::printf("{\"role\": \"sub\", \"stage\": \"RECEIVING\"}\n");
    std::fflush(stdout);

    // 事件驱动 + 取空为止：固定 sleep 轮询在高速率下取数跟不上，队列会单调积压。
    dds_entity_t waitset = dds_create_waitset(pp);
    if (waitset < 0) return 1;
    // 只对「有数据」敏感：若把 matched 等常驻状态也纳入唤醒源，waitset_wait 会
    // 立即返回，退化成每秒数百万次的空转（实测 empty_wakeups 达 6.28e6）。
    if (dds_set_status_mask(reader, DDS_DATA_AVAILABLE_STATUS) != DDS_RETCODE_OK) return 1;
    if (dds_waitset_attach(waitset, reader, reader) < 0) return 1;

    std::vector<double> lat_us;
    std::vector<long long> recv_ns;

    // 所有超时一律用 steady_clock 墙钟：
    //   1) waitset 返回次数 ≠ 经过时间，按次数计数会在毫秒内耗尽
    //   2) 单调时钟不受系统时间跳变影响
    using Clock = std::chrono::steady_clock;
    const auto t_start_clk = Clock::now();
    const auto overall_tp = std::chrono::milliseconds(opt.overall_ms);
    const auto idle_tp = std::chrono::milliseconds(opt.idle_ms);
    const auto first_tp = std::chrono::milliseconds(opt.first_packet_ms);
    const long long duration_deadline =
        opt.duration_s > 0 ? now_ns() + static_cast<long long>(opt.duration_s * 1e9) : 0;
    auto last_rx = t_start_clk;
    bool received_any = false;
    const char* end_reason = "overall_timeout";

    int empty_wakeups = 0;  // 只用于观测空唤醒，不参与退出判定
    int got = 0;
    long long t_last = 0;
    int batches = 0;
    int max_batch = 0;
    // 序列号统计：不能只用 sent-got 当丢包率，要区分重复、乱序与区间内缺失
    std::vector<char> seen;  // seen[seq] = 是否收到过
    long long rx_unique = 0;
    long long rx_dup = 0;
    long long rx_ooo = 0;
    long long last_seq = -1;
    long long max_seq = -1;

    while (true) {
        if (opt.count >= 0 && got >= opt.count) { end_reason = "count_reached"; break; }
        if (duration_deadline > 0 && now_ns() > duration_deadline) {
            end_reason = "duration_reached";
            break;
        }

        const auto now = Clock::now();
        if (now - t_start_clk >= overall_tp) { end_reason = "overall_timeout"; break; }

        int wait_ms = 200;
        if (!received_any) {
            // 首包窗口：收到第一个有效样本之前，Idle 计时不启动，
            // 否则「发布端还没开始发」会被误判成「接收已完成」。
            const auto left = first_tp - (now - t_start_clk);
            if (left <= std::chrono::milliseconds::zero()) {
                end_reason = "first_packet_timeout";
                break;
            }
            wait_ms = std::min<int>(
                wait_ms,
                static_cast<int>(
                    std::chrono::duration_cast<std::chrono::milliseconds>(left).count()));
        } else {
            const auto left = idle_tp - (now - last_rx);
            if (left <= std::chrono::milliseconds::zero()) {
                end_reason = "idle_timeout";
                break;
            }
            wait_ms = std::min<int>(
                wait_ms,
                static_cast<int>(
                    std::chrono::duration_cast<std::chrono::milliseconds>(left).count()));
        }
        if (wait_ms < 1) wait_ms = 1;

        // 取数频率不足会造成 Reader History 积压，与「取后处理耗时」是两个不同变量
        if (opt.take_interval_ms > 0) dds_sleepfor(DDS_MSECS(opt.take_interval_ms));

        dds_attach_t triggered[1];
        const int woken = dds_waitset_wait(waitset, triggered, 1, DDS_MSECS(wait_ms));

        // idle 只能在真的收到有效样本时清零：waitset 也会因 dispose/invalid_data
        // 等状态变化唤醒而取不到数据，在那里清零会空转不退出。
        const int got_before = got;
        if (woken > 0) {
            int batch = 0;
            for (;;) {
                void* samples[16] = {nullptr};
                dds_sample_info_t info[16];
                const int n = dds_take(reader, samples, info, 16, 16);
                if (n <= 0) break;
                for (int k = 0; k < n; ++k) {
                    if (!info[k].valid_data) continue;
                    auto* msg =
                        static_cast<motionslam_msgs_msg_dds__RuntimeEnvelope_*>(samples[k]);
                    t_last = now_ns();
                    const long long seq = static_cast<long long>(msg->channel_id);
                    if (seq >= static_cast<long long>(seen.size()))
                        seen.resize(static_cast<size_t>(seq) + 1, 0);
                    if (seen[static_cast<size_t>(seq)]) {
                        ++rx_dup;
                    } else {
                        seen[static_cast<size_t>(seq)] = 1;
                        ++rx_unique;
                        if (seq > max_seq) max_seq = seq;
                    }
                    if (seq < last_seq) ++rx_ooo;
                    last_seq = seq;
                    lat_us.push_back(static_cast<double>(t_last - msg->stamp_ns) / 1000.0);
                    recv_ns.push_back(t_last);
                    ++got;
                    ++batch;
                }
                dds_return_loan(reader, samples, n);
            }
            if (batch > 0) { ++batches; max_batch = std::max(max_batch, batch); }
            // 人为处理耗时：放在「取完之后、下次等待之前」，模拟消费能力不足
            if (opt.consumer_delay_ms > 0 && batch > 0) {
                dds_sleepfor(DDS_MSECS(opt.consumer_delay_ms));
            }
        }
        if (got == got_before) {
            ++empty_wakeups;
        } else {
            received_any = true;
            last_rx = Clock::now();
        }
    }
    dds_waitset_detach(waitset, reader);
    dds_delete(waitset);

    const long long t_start_ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>(t_start_clk.time_since_epoch()).count();
    const long long bucket_ns = static_cast<long long>(opt.bucket_s) * 1000000000LL;
    std::vector<double> head, tail;
    for (size_t i = 0; i < lat_us.size(); ++i) {
        if (recv_ns[i] - t_start_ns <= bucket_ns) head.push_back(lat_us[i]);
        if (t_last - recv_ns[i] <= bucket_ns) tail.push_back(lat_us[i]);
    }
    const double secs = static_cast<double>(t_last - t_start_ns) / 1e9;
    const double mb = static_cast<double>(got) * opt.size / (1024.0 * 1024.0);
    long long missing_in_range = 0;
    for (long long i = 0; i <= max_seq && i < static_cast<long long>(seen.size()); ++i)
        if (!seen[static_cast<size_t>(i)]) ++missing_in_range;

    // 时间语义必须可核验：退出原因、真实空闲时长、最后一条样本的相对时刻
    const long long idle_duration_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - last_rx).count();
    const long long last_rx_ts_ms =
        got > 0
            ? std::chrono::duration_cast<std::chrono::milliseconds>(last_rx - t_start_clk).count()
            : -1;
    const char* final_stage = (got > 0) ? "DONE" : "NO_DATA";
    std::printf(
        "{\"role\": \"sub\", \"stage\": \"%s\", \"valid\": %s, \"end_reason\": \"%s\", "
        "\"idle_duration_ms\": %lld, \"last_rx_ts_ms\": %lld, \"empty_wakeups\": %d, "
        "\"matched\": %d, \"got\": %d, \"rx_unique\": %lld, \"rx_dup\": %lld, "
        "\"rx_out_of_order\": %lld, \"missing_in_range\": %lld, \"max_seq\": %lld, "
        "\"size\": %d, \"seconds\": %.1f, \"msg_per_s\": %.0f, \"mb_per_s\": %.1f, "
        "\"take_batches\": %d, \"max_batch\": %d, \"consumer_delay_ms\": %d, "
        "\"take_interval_ms\": %d, "
        "\"end_to_end_us\": {\"p50\": %.1f, \"p95\": %.1f, \"p99\": %.1f, \"max\": %.1f}, "
        "\"head_window\": {\"n\": %zu, \"p50\": %.1f, \"p95\": %.1f}, "
        "\"tail_window\": {\"n\": %zu, \"p50\": %.1f, \"p95\": %.1f}, ",
        final_stage, (got > 0) ? "true" : "false", end_reason, idle_duration_ms, last_rx_ts_ms,
        empty_wakeups, matched, got, rx_unique, rx_dup, rx_ooo, missing_in_range, max_seq, opt.size,
        secs, static_cast<double>(got) / (secs > 0 ? secs : 1), mb / (secs > 0 ? secs : 1), batches,
        max_batch, opt.consumer_delay_ms, opt.take_interval_ms, pct(lat_us, 0.50),
        pct(lat_us, 0.95), pct(lat_us, 0.99), max_of(lat_us), head.size(), pct(head, 0.50),
        pct(head, 0.95), tail.size(), pct(tail, 0.50), pct(tail, 0.95));
    std::fflush(stdout);
    // 丢失即失败：让调用方无法把不完整的运行当成通过
    return (got > 0 && got == rx_unique && missing_in_range == 0 &&
            (opt.count < 0 || got == opt.count))
               ? 0
               : 1;
}

}  // namespace

int main(int argc, char** argv) {
    Options opt;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        auto next = [&]() -> std::string { return (i + 1 < argc) ? argv[++i] : std::string(); };
        if (arg == "--role") {
            opt.role = next();
        } else if (arg == "--topic") {
            opt.topic = next();
        } else if (arg == "--count") {
            opt.count = std::atoi(next().c_str());
        } else if (arg == "--size") {
            opt.size = std::atoi(next().c_str());
        } else if (arg == "--duration-s") {
            opt.duration_s = std::atof(next().c_str());
            if (opt.count < 0) opt.count = -1;
        } else if (arg == "--rate-hz") {
            opt.rate_hz = std::atof(next().c_str());
        } else if (arg == "--bucket-s") {
            opt.bucket_s = std::atoi(next().c_str());
        } else if (arg == "--reliability") {
            opt.reliability = next();
        } else if (arg == "--durability") {
            opt.durability = next();
        } else if (arg == "--depth") {
            opt.depth = std::atoi(next().c_str());
        } else if (arg == "--max-blocking-ms") {
            opt.max_blocking_ms = std::atoi(next().c_str());
        } else if (arg == "--default-qos") {
            opt.default_qos = true;
        } else if (arg == "--no-topic-qos") {
            opt.topic_qos = false;
        } else if (arg == "--take-interval-ms") {
            opt.take_interval_ms = std::atoi(next().c_str());
        } else if (arg == "--consumer-delay-ms") {
            opt.consumer_delay_ms = std::atoi(next().c_str());
        } else if (arg == "--await-match-ms") {
            opt.await_match_ms = std::atoi(next().c_str());
        } else if (arg == "--first-packet-ms") {
            opt.first_packet_ms = std::atoi(next().c_str());
        } else if (arg == "--overall-ms") {
            opt.overall_ms = std::atoi(next().c_str());
        } else if (arg == "--delay-after-match-ms") {
            opt.delay_after_match_ms = std::atoi(next().c_str());
        } else if (arg == "--pause-after") {
            opt.pause_after = std::atoi(next().c_str());
        } else if (arg == "--pause-ms") {
            opt.pause_ms = std::atoi(next().c_str());
        } else if (arg == "--idle-ms") {
            opt.idle_ms = std::atoi(next().c_str());
        } else {
            std::fprintf(stderr, "unknown arg: %s\n", arg.c_str());
            return 2;
        }
    }
    if (opt.count < 0 && opt.duration_s <= 0) {
        std::fprintf(stderr, "需要 --count 或 --duration-s\n");
        return 2;
    }
    // 非法值必须直接拒绝：静默退化会让整轮实验结论无效。
    if (opt.reliability != "reliable" && opt.reliability != "best_effort") {
        std::fprintf(stderr, "--reliability 只接受 reliable|best_effort，收到: %s\n",
                     opt.reliability.c_str());
        return 2;
    }
    if (opt.durability != "volatile" && opt.durability != "transient_local") {
        std::fprintf(stderr, "--durability 只接受 volatile|transient_local，收到: %s\n",
                     opt.durability.c_str());
        return 2;
    }
    if (opt.depth < 1) {
        std::fprintf(stderr, "--depth 必须 >= 1，收到: %d\n", opt.depth);
        return 2;
    }
    if (opt.max_blocking_ms < 0) {
        std::fprintf(stderr, "--max-blocking-ms 不能为负，收到: %d\n", opt.max_blocking_ms);
        return 2;
    }

    dds_entity_t pp = dds_create_participant(DDS_DOMAIN_DEFAULT, nullptr, nullptr);
    if (pp < 0) return 1;
    dds_qos_t* qos = make_qos(opt);
    const int rc = opt.role == "pub" ? run_pub(opt, pp, qos) : run_sub(opt, pp, qos);
    dds_delete_qos(qos);
    dds_delete(pp);
    return rc;
}
