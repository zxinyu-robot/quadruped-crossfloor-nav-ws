# 任务在当前世界代数上的绑定（执行只允许单一代际切片）
string task_id
uint64 task_revision
WorldRef bound_world_ref
uint64 lease_epoch     # 执行租约纪元
string plan_id
int64 traj_id          # 当前有效轨迹 ID（-1 表示无）
