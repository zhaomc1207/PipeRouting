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
- 弯曲规则字段：`min_bend_radius_required` / `min_bend_radius_observed` / `bend_rule_violation_count` / `bend_rule_violations`

## 12. 最小弯曲半径检查说明
- 当前实现基于路径连续三点的几何估算（外接圆半径）。
- 这是 MVP 级制造规则检查，不等价于真实弯管工艺仿真。
- 后续可替换为更严格的曲线/样条曲率连续检查。

## 13. 局部重算（MVP）
- 当前提供 MVP 级 local reroute，入口在 `pipe_routing/local_reroute.py`。
- `changed_region` 第一版只支持 `type=box`。
- 未受影响管路保持原路径不变，并作为动态障碍物参与受影响管路重算。
- 受影响管路重新运行寻路、平滑与规则检查。

## 15. Simplified CBS（MVP）
- 底层单管寻路器仍是 3D A*。
- simplified CBS 是上层冲突回退协调器：检测最终冲突后，生成 box 约束并尝试对冲突管路回退重算。
- 当前为 MVP，不是完整 CBS 搜索树实现。
- 具有 `max_iterations` 迭代上限，避免死循环。
- 冲突约束目前使用 box 近似。

## 16. 多场景 Case 批量运行
- 目录：`data/cases/`
- 脚本：`python run_cases.py`
- 输出：`outputs/cases/<case_name>/result.json`、`result.html`、`routing_report.md`
- 汇总：`outputs/cases/summary.csv`

当前内置场景：
1. `case_01_simple_clear`: 无障碍基础可达性
2. `case_02_obstacle_detour`: 单障碍绕行
3. `case_03_multi_pipe_conflict`: 多管近邻冲突压力
4. `case_04_clamp_guided`: 含 clamp 候选点的路径质量观察
5. `case_05_local_reroute`: 适合局部重算演示
6. `case_06_dense_obstacles`: 密集障碍复杂路径
7. `case_07_no_solution`: 无解场景（用于失败处理验证）

## 14. 3D 可视化说明
- `result.html` 中管路按 `pipe.diameter` 渲染为近似 tube（分段圆柱 mesh）。
- tube 是可视化近似，不等于真实 CAD 管路实体。
- 默认使用稳定的连续 tube 可视化（不强制圆角弯头）。
- rounded elbow 是实验性可选功能，默认关闭。
- 当前阶段不追求 CAD 级圆角弯头。

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

## Optimization Priority (Current Phase)
1. Feasible path first
2. Obstacle avoidance and pipe-pipe conflict minimization
3. Bend radius compliance and smoothness
4. Clamp guidance as soft preference
5. Path length as reference only (weak objective)

## Pipe-specific Clamp Candidates
- `clamp_candidates` are soft constraints, not mandatory waypoints.
- `applies_to` lists which pipe ids a clamp belongs to.
- During routing, each pipe only receives clamp reward from its own clamps (`applies_to` contains that pipe), or global clamps when `applies_to` is missing/empty.
- If clamp passing would violate obstacle/conflict safety, planner may skip it.
- `required_clamps` is a future hard-constraint extension and is not implemented in current phase.

### Case Output Naming
- Recommended files are prefixed by case name: outputs/cases/<case_name>/<case_name>_result.json, <case_name>_result.html, <case_name>_routing_report.md.
- Legacy esult.json, esult.html, outing_report.md are still generated as compatibility copies in the same case folder.
- outputs/cases/summary.csv includes esult_json_path, esult_html_path, outing_report_path pointing to prefixed files.

