# 统一世界引用 / 版本向量（代数变化的单一权威身份）
# 用于任务绑定、重绑定事务与派生件代际校验。
string asset_id        # 空间资产 ID
string map_id
string floor_id
string tile_id
uint64 revision        # 地图资产版本
uint64 context_generation  # PGO/建图上下文代数
string frame_id        # world / map
string digest          # 资产内容摘要（确定性哈希）
