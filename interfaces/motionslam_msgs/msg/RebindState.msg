# 任务级重绑定生命周期状态（实时性 / 生命周期状态强调）
string task_id
uint8 state            # 0=BOUND 1=INVALIDATED 2=REBIND_PENDING 3=REBIND_READY 4=RESUMED 5=REBIND_REJECTED
WorldRef active_world_ref
uint64 lease_epoch
string detail
