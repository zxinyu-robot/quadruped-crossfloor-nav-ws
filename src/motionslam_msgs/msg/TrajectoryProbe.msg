# 【仅传输能力探针，不是符号面契约】
# 本消息只用于验证 DDS 能否承载「嵌套标准消息 + 自定义消息数组 + string + 原生数组」
# 这类复杂结构，不参与任何业务语义，也不代表 runtime_core 会向导航栈传递轨迹。
# 按三空间约束（OS0）：执行只发生在度量空间，符号面不得承载几何量或速度指令。
# 真正的符号面契约是 motionslam.task_intent.v1 / motionslam.execution_feedback.v1。
std_msgs/Header header
string proposal_id
string model_id
float64[] stamps_s
PosePoint[] points
uint8 kind
bool ok
