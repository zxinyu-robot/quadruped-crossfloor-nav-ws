// DDS 同机吞吐/时延基准（用于验证 CycloneDDS 的 SHM 是否真的生效）。
//
// 两个进程跑在同一台机器上：pub 发 N 条定长载荷，sub 收并统计。
// 载荷用 RuntimeEnvelope.payload 承载，stamp_ns 写发送时刻，sub 据此算单程时延。
//
// 用法：
//   dds_bench --role sub --count 2000 --size 1048576 --topic rt/bench/envelope
//   dds_bench --role pub --count 2000 --size 1048576 --topic rt/bench/envelope
//
// 输出：单行 JSON（便于脚本解析与报告生成）。
#include <dds/dds.h>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
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
    int count = 1000;
    int size = 65536;
    bool best_effort = false;
    int timeout_ms = 20000;
};

dds_qos_t* make_qos(const Options& opt) {
    dds_qos_t* qos = dds_create_qos();
    dds_qset_reliability(
        qos, opt.best_effort ? DDS_RELIABILITY_BEST_EFFORT : DDS_RELIABILITY_RELIABLE,
        DDS_MSECS(200));
    dds_qset_history(qos, DDS_HISTORY_KEEP_LAST, static_cast<int32_t>(opt.count + 8));
    dds_qset_durability(qos, DDS_DURABILITY_VOLATILE);
    return qos;
}

void set_payload(motionslam_msgs_msg_dds__RuntimeEnvelope_& msg, int size, long long stamp) {
    std::memset(&msg, 0, sizeof msg);
    msg.schema_version = 1;
    msg.channel_id = 1;
    msg.stamp_ns = stamp;
    msg.payload._length = static_cast<uint32_t>(size);
    msg.payload._buffer = static_cast<uint8_t*>(dds_alloc(static_cast<size_t>(size)));
    std::memset(msg.payload._buffer, 0x5a, static_cast<size_t>(size));
    msg.payload._release = true;
}

int run_pub(const Options& opt, dds_entity_t pp, dds_qos_t* qos) {
    dds_entity_t topic = dds_create_topic(
        pp, &motionslam_msgs_msg_dds__RuntimeEnvelope__desc, opt.topic.c_str(), qos, nullptr);
    if (topic < 0) return 1;
    dds_entity_t writer = dds_create_writer(pp, topic, qos, nullptr);
    if (writer < 0) return 1;

    // 等订阅方就绪
    for (int i = 0; i < opt.timeout_ms / 50; ++i) {
        dds_publication_matched_status_t st;
        if (dds_get_publication_matched_status(writer, &st) == DDS_RETCODE_OK &&
            st.current_count > 0) {
            break;
        }
        dds_sleepfor(DDS_MSECS(50));
    }

    dds_publication_matched_status_t ms;
    dds_get_publication_matched_status(writer, &ms);
    std::fprintf(stderr, "matched=%u\n", (unsigned)ms.current_count);

    const long long t0 = now_ns();
    int sent = 0;
    for (int i = 0; i < opt.count; ++i) {
        motionslam_msgs_msg_dds__RuntimeEnvelope_ msg;
        set_payload(msg, opt.size, now_ns());
        if (dds_write(writer, &msg) != DDS_RETCODE_OK) break;
        ++sent;
        dds_free(msg.payload._buffer);
    }
    const long long t1 = now_ns();
    // 等数据真正发完再退出，否则 delete participant 会丢掉未确认的样本
    dds_write_flush(writer);
    dds_sleepfor(DDS_MSECS(800));
    const double secs = static_cast<double>(t1 - t0) / 1e9;
    const double mb = static_cast<double>(sent) * opt.size / (1024.0 * 1024.0);
    std::printf(
        "{\"role\": \"pub\", \"sent\": %d, \"size\": %d, \"seconds\": %.3f, "
        "\"msg_per_s\": %.0f, \"mb_per_s\": %.1f}\n",
        sent, opt.size, secs, sent / (secs > 0 ? secs : 1),
        mb / (secs > 0 ? secs : 1));
    return 0;
}

int run_sub(const Options& opt, dds_entity_t pp, dds_qos_t* qos) {
    dds_entity_t topic = dds_create_topic(
        pp, &motionslam_msgs_msg_dds__RuntimeEnvelope__desc, opt.topic.c_str(), qos, nullptr);
    if (topic < 0) return 1;
    dds_entity_t reader = dds_create_reader(pp, topic, qos, nullptr);
    if (reader < 0) return 1;
    std::printf("{\"role\": \"sub\", \"ready\": true}\n");
    std::fflush(stdout);

    std::vector<double> lat_us;
    lat_us.reserve(static_cast<size_t>(opt.count));
    long long t_first = 0;
    long long t_last = 0;
    int got = 0;
    const int idle_limit = opt.timeout_ms / 2;  // 2ms/次 → 约 timeout_ms 的空闲上限
    int idle = 0;
    while (got < opt.count && idle < idle_limit) {
        void* samples[16] = {nullptr};
        dds_sample_info_t info[16];
        const int n = dds_take(reader, samples, info, 16, 16);
        if (n <= 0) {
            dds_sleepfor(DDS_MSECS(2));
            ++idle;
            continue;
        }
        idle = 0;
        for (int k = 0; k < n && got < opt.count; ++k) {
            if (!info[k].valid_data) continue;
            auto* msg = static_cast<motionslam_msgs_msg_dds__RuntimeEnvelope_*>(samples[k]);
            t_last = now_ns();
            if (got == 0) t_first = t_last;
            lat_us.push_back(static_cast<double>(t_last - msg->stamp_ns) / 1000.0);
            ++got;
        }
        dds_return_loan(reader, samples, n);
    }
    const double secs = static_cast<double>(t_last - t_first) / 1e9;

    auto pct = [&](double p) -> double {
        if (lat_us.empty()) return 0.0;
        std::vector<double> v = lat_us;
        const size_t idx = static_cast<size_t>(p * (v.size() - 1));
        std::nth_element(v.begin(), v.begin() + static_cast<long>(idx), v.end());
        return v[idx];
    };
    const double mb = static_cast<double>(got) * opt.size / (1024.0 * 1024.0);
    std::printf(
        "{\"role\": \"sub\", \"got\": %d, \"size\": %d, \"seconds\": %.3f, "
        "\"msg_per_s\": %.0f, \"mb_per_s\": %.1f, "
        "\"latency_us\": {\"p50\": %.0f, \"p95\": %.0f, \"p99\": %.0f}}\n",
        got, opt.size, secs, got / (secs > 0 ? secs : 1), mb / (secs > 0 ? secs : 1),
        pct(0.50), pct(0.95), pct(0.99));
    return got == opt.count ? 0 : 1;
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
        } else if (arg == "--best-effort") {
            opt.best_effort = true;
        } else if (arg == "--timeout-ms") {
            opt.timeout_ms = std::atoi(next().c_str());
        } else {
            std::fprintf(stderr, "unknown arg: %s\n", arg.c_str());
            return 2;
        }
    }

    dds_entity_t pp = dds_create_participant(DDS_DOMAIN_DEFAULT, nullptr, nullptr);
    if (pp < 0) return 1;
    dds_qos_t* qos = make_qos(opt);
    const int rc = opt.role == "pub" ? run_pub(opt, pp, qos) : run_sub(opt, pp, qos);
    dds_delete_qos(qos);
    dds_delete(pp);
    return rc;
}
