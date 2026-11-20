// DDS 互通实验工具（MVP）：原生 CycloneDDS 参与者 ⇄ ROS 2 导航栈，双向收发。
//
// 只依赖 libddsc 与由 .msg 生成的类型支持，不依赖 rcl / rclcpp / rmw。
// 用法：
//   interop_native --type envelope|probe --role pub|sub [--topic T] [--count N] [--timeout-ms M]
//
// pub：发一条固定载荷，打印 matched_subscribers 后退出（供 ROS 侧 echo 校验）
// sub：等待 N 条消息，逐条打印 JSON，超时则以非零码退出
#include <dds/dds.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include "motionslam_msgs/msg/RuntimeEnvelope.h"
#include "motionslam_msgs/msg/TrajectoryProbe.h"

namespace {

constexpr int kDefaultTimeoutMs = 15000;

struct Options {
    std::string type = "envelope";
    std::string role = "pub";
    std::string topic = "rt/motionslam/interop/envelope";
    int count = 1;
    int timeout_ms = kDefaultTimeoutMs;
};

dds_qos_t* make_qos() {
    dds_qos_t* qos = dds_create_qos();
    dds_qset_reliability(qos, DDS_RELIABILITY_RELIABLE, DDS_SECS(2));
    dds_qset_history(qos, DDS_HISTORY_KEEP_LAST, 10);
    dds_qset_durability(qos, DDS_DURABILITY_VOLATILE);
    return qos;
}

int wait_matched(dds_entity_t writer, int timeout_ms) {
    const int steps = timeout_ms / 50;
    for (int i = 0; i < steps; ++i) {
        dds_publication_matched_status_t st;
        if (dds_get_publication_matched_status(writer, &st) == DDS_RETCODE_OK &&
            st.current_count > 0) {
            return static_cast<int>(st.current_count);
        }
        dds_sleepfor(DDS_MSECS(50));
    }
    return 0;
}

void set_string(char*& dst, const char* value) {
    const size_t n = std::strlen(value) + 1;
    dst = static_cast<char*>(dds_alloc(n));
    std::memcpy(dst, value, n);
}

// ---------------------------------------------------------------- envelope

int pub_envelope(const Options& opt, dds_entity_t pp, dds_qos_t* qos) {
    dds_entity_t topic = dds_create_topic(
        pp, &motionslam_msgs_msg_dds__RuntimeEnvelope__desc, opt.topic.c_str(), qos, nullptr);
    if (topic < 0) return 1;
    dds_entity_t writer = dds_create_writer(pp, topic, qos, nullptr);
    if (writer < 0) return 1;

    const int matched = wait_matched(writer, opt.timeout_ms);
    std::printf("{\"matched_subscribers\": %d}\n", matched);
    std::fflush(stdout);

    const char* payload = "{\"event\":\"interop\",\"from\":\"native\"}";
    motionslam_msgs_msg_dds__RuntimeEnvelope_ msg;
    std::memset(&msg, 0, sizeof msg);
    msg.schema_version = 1;
    msg.channel_id = 7;
    msg.stamp_ns = 1700000000123456789LL;
    msg.payload._length = static_cast<uint32_t>(std::strlen(payload));
    msg.payload._buffer = static_cast<uint8_t*>(dds_alloc(msg.payload._length));
    std::memcpy(msg.payload._buffer, payload, msg.payload._length);
    msg.payload._release = true;

    const dds_return_t rc = dds_write(writer, &msg);
    std::printf("{\"write_rc\": %d}\n", static_cast<int>(rc));
    std::fflush(stdout);
    dds_sleepfor(DDS_MSECS(500));
    return rc == DDS_RETCODE_OK ? 0 : 1;
}

int sub_envelope(const Options& opt, dds_entity_t pp, dds_qos_t* qos) {
    dds_entity_t topic = dds_create_topic(
        pp, &motionslam_msgs_msg_dds__RuntimeEnvelope__desc, opt.topic.c_str(), qos, nullptr);
    if (topic < 0) return 1;
    dds_entity_t reader = dds_create_reader(pp, topic, qos, nullptr);
    if (reader < 0) return 1;

    int got = 0;
    const int steps = opt.timeout_ms / 20;
    for (int i = 0; i < steps && got < opt.count; ++i) {
        void* samples[4] = {nullptr};
        dds_sample_info_t info[4];
        const int n = dds_take(reader, samples, info, 4, 4);
        for (int k = 0; k < n && got < opt.count; ++k) {
            if (!info[k].valid_data) continue;
            auto* msg = static_cast<motionslam_msgs_msg_dds__RuntimeEnvelope_*>(samples[k]);
            std::string payload;
            if (msg->payload._buffer != nullptr) {
                payload.assign(reinterpret_cast<const char*>(msg->payload._buffer),
                               msg->payload._length);
            }
            std::printf(
                "{\"schema_version\": %u, \"channel_id\": %u, \"stamp_ns\": %lld, "
                "\"payload\": \"%s\"}\n",
                msg->schema_version, msg->channel_id, static_cast<long long>(msg->stamp_ns),
                payload.c_str());
            std::fflush(stdout);
            ++got;
        }
        dds_return_loan(reader, samples, n > 0 ? n : 0);
        if (got < opt.count) dds_sleepfor(DDS_MSECS(20));
    }
    if (got < opt.count) {
        std::fprintf(stderr, "timeout: got %d/%d\n", got, opt.count);
        return 1;
    }
    return 0;
}

// ---------------------------------------------------------------- probe

void fill_probe(motionslam_msgs_msg_dds__TrajectoryProbe_& msg) {
    std::memset(&msg, 0, sizeof msg);
    msg.header.stamp.sec = 1234;
    msg.header.stamp.nanosec = 5678u;
    set_string(msg.header.frame_id, "map");
    set_string(msg.proposal_id, "probe-42");
    set_string(msg.model_id, "stub-vlm");

    msg.stamps_s._length = 3;
    msg.stamps_s._buffer = static_cast<double*>(dds_alloc(3 * sizeof(double)));
    msg.stamps_s._buffer[0] = 0.1;
    msg.stamps_s._buffer[1] = 0.2;
    msg.stamps_s._buffer[2] = 0.3;
    msg.stamps_s._release = true;

    msg.points._length = 2;
    msg.points._buffer = static_cast<motionslam_msgs_msg_dds__PosePoint_*>(
        dds_alloc(2 * sizeof(motionslam_msgs_msg_dds__PosePoint_)));
    std::memset(msg.points._buffer, 0, 2 * sizeof(motionslam_msgs_msg_dds__PosePoint_));
    msg.points._buffer[0].x = 1.5;
    msg.points._buffer[0].y = -1.25;
    msg.points._buffer[0].z = 0.35;
    msg.points._buffer[0].yaw_deg = 90.0;
    msg.points._buffer[1].x = 2.5;
    msg.points._buffer[1].y = -1.25;
    msg.points._buffer[1].z = 0.35;
    msg.points._buffer[1].yaw_deg = 91.5;
    msg.points._release = true;

    msg.kind = 7;
    msg.ok = true;
}

void print_probe(const motionslam_msgs_msg_dds__TrajectoryProbe_& msg) {
    std::printf(
        "{\"frame_id\": \"%s\", \"stamp\": {\"sec\": %d, \"nanosec\": %u}, "
        "\"proposal_id\": \"%s\", \"model_id\": \"%s\", \"stamps_s\": [",
        msg.header.frame_id ? msg.header.frame_id : "", msg.header.stamp.sec,
        msg.header.stamp.nanosec, msg.proposal_id ? msg.proposal_id : "",
        msg.model_id ? msg.model_id : "");
    for (uint32_t i = 0; i < msg.stamps_s._length; ++i) {
        std::printf("%s%.3f", i ? ", " : "", msg.stamps_s._buffer[i]);
    }
    std::printf("], \"points\": [");
    for (uint32_t i = 0; i < msg.points._length; ++i) {
        const auto& p = msg.points._buffer[i];
        std::printf("%s{\"x\": %.3f, \"y\": %.3f, \"z\": %.3f, \"yaw_deg\": %.3f}",
                    i ? ", " : "", p.x, p.y, p.z, p.yaw_deg);
    }
    std::printf("], \"kind\": %u, \"ok\": %s}\n", msg.kind, msg.ok ? "true" : "false");
    std::fflush(stdout);
}

int pub_probe(const Options& opt, dds_entity_t pp, dds_qos_t* qos) {
    dds_entity_t topic = dds_create_topic(
        pp, &motionslam_msgs_msg_dds__TrajectoryProbe__desc, opt.topic.c_str(), qos, nullptr);
    if (topic < 0) return 1;
    dds_entity_t writer = dds_create_writer(pp, topic, qos, nullptr);
    if (writer < 0) return 1;

    const int matched = wait_matched(writer, opt.timeout_ms);
    std::printf("{\"matched_subscribers\": %d}\n", matched);
    std::fflush(stdout);

    motionslam_msgs_msg_dds__TrajectoryProbe_ msg;
    fill_probe(msg);
    const dds_return_t rc = dds_write(writer, &msg);
    std::printf("{\"write_rc\": %d}\n", static_cast<int>(rc));
    std::fflush(stdout);
    dds_free(msg.header.frame_id);
    dds_free(msg.proposal_id);
    dds_free(msg.model_id);
    dds_free(msg.stamps_s._buffer);
    dds_free(msg.points._buffer);
    dds_sleepfor(DDS_MSECS(500));
    return rc == DDS_RETCODE_OK ? 0 : 1;
}

int sub_probe(const Options& opt, dds_entity_t pp, dds_qos_t* qos) {
    dds_entity_t topic = dds_create_topic(
        pp, &motionslam_msgs_msg_dds__TrajectoryProbe__desc, opt.topic.c_str(), qos, nullptr);
    if (topic < 0) return 1;
    dds_entity_t reader = dds_create_reader(pp, topic, qos, nullptr);
    if (reader < 0) return 1;

    int got = 0;
    const int steps = opt.timeout_ms / 20;
    for (int i = 0; i < steps && got < opt.count; ++i) {
        void* samples[4] = {nullptr};
        dds_sample_info_t info[4];
        const int n = dds_take(reader, samples, info, 4, 4);
        for (int k = 0; k < n && got < opt.count; ++k) {
            if (!info[k].valid_data) continue;
            print_probe(*static_cast<motionslam_msgs_msg_dds__TrajectoryProbe_*>(samples[k]));
            ++got;
        }
        dds_return_loan(reader, samples, n > 0 ? n : 0);
        if (got < opt.count) dds_sleepfor(DDS_MSECS(20));
    }
    if (got < opt.count) {
        std::fprintf(stderr, "timeout: got %d/%d\n", got, opt.count);
        return 1;
    }
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
    Options opt;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        auto next = [&]() -> std::string { return (i + 1 < argc) ? argv[++i] : std::string(); };
        if (arg == "--type") {
            opt.type = next();
        } else if (arg == "--role") {
            opt.role = next();
        } else if (arg == "--topic") {
            opt.topic = next();
        } else if (arg == "--count") {
            opt.count = std::atoi(next().c_str());
        } else if (arg == "--timeout-ms") {
            opt.timeout_ms = std::atoi(next().c_str());
        } else {
            std::fprintf(stderr, "unknown arg: %s\n", arg.c_str());
            return 2;
        }
    }
    if (opt.topic.rfind("rt/", 0) != 0) {
        std::fprintf(stderr,
                     "error: topic 必须以 rt/ 开头（ROS 侧 /foo 在 DDS 上是 rt/foo），当前：%s\n",
                     opt.topic.c_str());
        return 2;
    }

    dds_entity_t pp = dds_create_participant(DDS_DOMAIN_DEFAULT, nullptr, nullptr);
    if (pp < 0) {
        std::fprintf(stderr, "participant error: %s\n", dds_strretcode(-pp));
        return 1;
    }
    dds_qos_t* qos = make_qos();

    int rc;
    const bool pub = opt.role == "pub";
    if (opt.type == "envelope") {
        rc = pub ? pub_envelope(opt, pp, qos) : sub_envelope(opt, pp, qos);
    } else if (opt.type == "probe") {
        rc = pub ? pub_probe(opt, pp, qos) : sub_probe(opt, pp, qos);
    } else {
        std::fprintf(stderr, "unknown type: %s\n", opt.type.c_str());
        rc = 2;
    }

    dds_delete_qos(qos);
    dds_delete(pp);
    return rc;
}
