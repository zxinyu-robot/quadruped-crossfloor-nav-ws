// 生成物的类型契约断言：m_typename 必须与 rmw_cyclonedds_cpp 公告的类型名一致。
// 只依赖 libddsc，不需要 ROS（可在任意机器上跑，用于 CI 门禁）。
#include <dds/dds.h>

#include <cstdio>
#include <cstring>
#include <string>

#include "motionslam_msgs/RuntimeEnvelope.h"

namespace {

int passed = 0;
#define CHECK(cond)                                                             \
    do {                                                                        \
        if (!(cond)) {                                                          \
            std::fprintf(stderr, "FAIL %s:%d %s\n", __FILE__, __LINE__, #cond); \
            return 1;                                                           \
        }                                                                       \
        ++passed;                                                               \
    } while (0)

constexpr const char* kExpectedTypeName = "motionslam_msgs::msg::dds_::RuntimeEnvelope_";
constexpr const char* kExpectedTopicName = "rt/runtime/envelope";

int test_descriptor_typename() {
    const dds_topic_descriptor_t& desc = motionslam_msgs_msg_dds__RuntimeEnvelope__desc;
    CHECK(desc.m_typename != nullptr);
    CHECK(std::strcmp(desc.m_typename, kExpectedTypeName) == 0);
    // 结构布局：uint32 + uint32 + sequence<octet> + int64
    CHECK(desc.m_nops == 5u);
    CHECK(desc.m_nkeys == 0u);
    return 0;
}

int test_topic_creation() {
    dds_entity_t pp = dds_create_participant(DDS_DOMAIN_DEFAULT, nullptr, nullptr);
    CHECK(pp >= 0);

    dds_qos_t* qos = dds_create_qos();
    dds_qset_reliability(qos, DDS_RELIABILITY_RELIABLE, DDS_SECS(1));
    dds_qset_history(qos, DDS_HISTORY_KEEP_LAST, 10);

    dds_entity_t tp = dds_create_topic(pp, &motionslam_msgs_msg_dds__RuntimeEnvelope__desc,
                                       kExpectedTopicName, qos, nullptr);
    CHECK(tp >= 0);

    dds_entity_t wr = dds_create_writer(pp, tp, qos, nullptr);
    CHECK(wr >= 0);
    dds_entity_t rd = dds_create_reader(pp, tp, qos, nullptr);
    CHECK(rd >= 0);

    // 同进程内 writer/reader 应能与自己匹配（sertype 自洽性检查）
    bool matched = false;
    for (int i = 0; i < 100; ++i) {
        dds_publication_matched_status_t st;
        if (dds_get_publication_matched_status(wr, &st) == 0 && st.current_count > 0) {
            matched = true;
            break;
        }
        dds_sleepfor(DDS_MSECS(20));
    }
    CHECK(matched);

    motionslam_msgs_msg_dds__RuntimeEnvelope_ msg;
    std::memset(&msg, 0, sizeof msg);
    msg.schema_version = 3;
    msg.channel_id = 42;
    msg.stamp_ns = 987654321;
    const char payload[] = "roundtrip";
    msg.payload._buffer = static_cast<uint8_t*>(dds_alloc(sizeof payload - 1));
    std::memcpy(msg.payload._buffer, payload, sizeof payload - 1);
    msg.payload._length = sizeof payload - 1;
    msg.payload._release = true;
    CHECK(dds_write(wr, &msg) == DDS_RETCODE_OK);

    void* samples[1] = {nullptr};
    dds_sample_info_t info[1];
    bool got = false;
    for (int i = 0; i < 100 && !got; ++i) {
        if (dds_take(rd, samples, info, 1, 1) > 0 && info[0].valid_data) {
            auto* got_msg = static_cast<motionslam_msgs_msg_dds__RuntimeEnvelope_*>(samples[0]);
            CHECK(got_msg->schema_version == 3);
            CHECK(got_msg->channel_id == 42);
            CHECK(got_msg->payload._length == sizeof payload - 1);
            CHECK(std::memcmp(got_msg->payload._buffer, payload, sizeof payload - 1) == 0);
            got = true;
            dds_return_loan(rd, samples, 1);
        }
        dds_sleepfor(DDS_MSECS(20));
    }
    CHECK(got);

    dds_delete(pp);
    dds_delete_qos(qos);
    return 0;
}

}  // namespace

int main() {
    if (test_descriptor_typename()) return 1;
    if (test_topic_creation()) return 1;
    std::printf("all %d checks passed\n", passed);
    return 0;
}
