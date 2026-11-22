# old→new 可审计重绑定记录（rebind 事务应答）
string task_id
WorldRef from_world_ref
WorldRef to_world_ref
string reason
uint32 policy
uint8 result           # 0=PENDING 1=READY 2=APPLIED 3=REJECTED
string[] invalidated_artifacts  # 被作废的旧代派生件（path/traj/token/subgraph/lease）
string detail
int64 stamp_ns
