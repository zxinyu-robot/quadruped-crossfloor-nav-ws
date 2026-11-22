# 重绑定事务请求（幂等键 = task_id + from_world_ref + to_world_ref）
string task_id
WorldRef from_world_ref
WorldRef to_world_ref
string reason          # 稳定原因码：pgo_applied / map_switch / epoch_bump ...
uint32 policy          # 0=AUTO 1=REQUIRE_APPROVAL
