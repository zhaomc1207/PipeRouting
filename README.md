# PipeRouting MVP

## 1. 项目目标
实现汽车前舱管路自动布置 MVP（算法原型）。

## 2. 当前 MVP 功能
- JSON 输入（`data/demo_case.json`）
- 3D grid
- box 障碍物与膨胀
- 单管 3D A*
- 多管顺序规划
- 已规划管路动态障碍物
- 管-管冲突检测
- `outputs/result.json` 输出
- Plotly 生成 `outputs/result.html`
- Markdown 报告 `outputs/routing_report.md`
- pytest 基础测试

## 3. Windows + conda 环境
```bash
conda create -n pipe-routing python=3.10 -y
conda activate pipe-routing
```

## 4. 安装依赖
```bash
pip install -r requirements.txt
```

## 5. 运行 demo
```bash
python run_demo.py
```

## 6. 运行测试
```bash
pytest -q
```

## 7. 输入 JSON 字段说明
- `workspace`: `min` / `max` / `resolution`
- `obstacles`: 仅支持 `type=box`，包含 `min` / `max`
- `pipes`: `start` / `end` / `diameter` / `clearance` / `min_bend_radius`
- `clamp_candidates`: `position` / `radius`

## 8. result.json 字段说明
每根管路包含：
- `id`
- `success`
- `path`
- `length`
- `bend_count`
- `conflict_count`
- `conflicts`
- `used_clamps`: 路径进入 clamp 半径范围的 clamp id 列表
- `min_distance_to_clamps`: 管路到每个 clamp 的最近距离（字典，key 为 clamp id）
- `raw_path`: A* 原始路径
- `smoothed_path`: 平滑后的候选路径
- `smoothing_applied` / `smoothing_reverted` / `smoothing_revert_reason`: 平滑是否生效、是否回退及原因
- `error`（失败时）

整体还包含：
- `conflicts`: 全局冲突列表

## 9. routing_report.md 说明
`outputs/routing_report.md` 由 `run_demo.py` 自动生成，包含：
- Summary: `total_pipes` / `success_count` / `failed_count` / `total_length` / `total_conflicts`
- 每根 pipe: `success` / `length` / `bend_count` / `conflict_count` / `used_clamps` / `min_distance_to_clamps` / `error`（失败时）
- 每根 pipe 还包含平滑信息：`smoothing_applied` / `smoothing_reverted` / `smoothing_revert_reason` / `raw_point_count` / `smoothed_point_count`

## 10. 当前限制
- 仅 JSON 输入
- 仅 box 障碍物
- 不接 CATIA / STP / STEP / OBJ / STL
- 不做 GUI、Web、数据库、ROS

## 11. 后续扩展方向
1. 固定点奖励增强和强制 waypoint
2. 更严格弯曲半径
3. 路径平滑
4. 局部重算
5. 简化 CBS 回退重算

打开可视化：
```powershell
start outputs\result.html
```
