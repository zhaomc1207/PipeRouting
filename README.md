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
- `error`（失败时）

整体还包含：
- `conflicts`: 全局冲突列表

## 9. 当前限制
- 仅 JSON 输入
- 仅 box 障碍物
- 不接 CATIA / STP / STEP / OBJ / STL
- 不做 GUI、Web、数据库、ROS

## 10. 后续扩展方向
1. 固定点奖励
2. 更严格弯曲半径
3. 路径平滑
4. 局部重算
5. 简化 CBS 回退重算

打开可视化：
```powershell
start outputs\result.html
```
