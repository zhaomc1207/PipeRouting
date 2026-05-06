# AGENTS.md

你是一个高级汽车布管算法工程师和 Python 工程师。

## 1. 项目目标

本项目目标是实现一个“汽车前舱管路自动布置 MVP”。

当前阶段只做算法原型，不做完整工业软件。

输入是一个简化 JSON 文件，里面包含：

- 三维工作空间 workspace
- 障碍物 obstacles
- 多根管路 pipes
- 候选固定点 clamp_candidates

输出是：

- `outputs/result.json`
- `outputs/result.html`

算法需要完成：

- 3D 空间自动寻路
- 障碍物避让
- 管径和安全间隙处理
- 多根管路规划
- 管路之间冲突检测
- 可视化展示

---

## 2. 运行环境

本项目运行环境是：

- Windows 本地电脑
- VS Code 终端
- conda 环境
- Python 3.10
- Codex 负责生成和修改代码

请保证所有代码和命令都能在 Windows + VS Code 终端 + conda 环境下运行。

不要依赖 Linux 专用命令。

所有路径处理请使用 `pathlib`，不要硬编码绝对路径。

---

## 3. 重要边界

请严格遵守以下边界：

1. 不接 CATIA。
2. 不读取 STP / STEP。
3. 不读取 OBJ / STL。
4. 不做 GUI。
5. 不用 ROS。
6. 不做 Web 服务。
7. 不使用数据库。
8. 不依赖 Linux 专用命令。
9. 只使用 Python。
10. 使用 Python 3.10。
11. 第一阶段只读取 JSON 输入。
12. 第一阶段只支持 box 障碍物。
13. 不要做过度复杂架构，优先保证 MVP 可运行、可测试、可扩展。

所有功能都要能通过下面命令运行：

```bash
python run_demo.py
```

测试命令统一使用：

```bash
pytest -q
```

依赖统一写入：

```text
requirements.txt
```

---

## 4. 业务背景

汽车前舱内有多根管路需要自动布置。

每根管路有：

- 起点 start
- 终点 end
- 管径 diameter
- 安全间隙 clearance
- 最小弯曲半径 min_bend_radius

空间内存在障碍物，例如：

- 发动机
- 电池
- 车身结构
- 其他零部件

算法需要为每根管路生成一条三维路径。

路径不能只追求最短，还要尽量满足：

- 不碰撞障碍物
- 与障碍物保持安全间隙
- 管路之间保持安全间隙
- 减少不必要的绕路
- 减少急转弯
- 尽量靠近候选固定点
- 结果方便工程师检查

---

## 5. 当前 MVP 范围

当前 MVP 只实现简化能力。

### 输入

只读取：

```text
data/demo_case.json
```

JSON 中包含：

```text
workspace
obstacles
pipes
clamp_candidates
```

### 输出

需要输出：

```text
outputs/result.json
outputs/result.html
```

`result.json` 中至少包含：

- 每根管子的 id
- success
- path
- length
- bend_count
- conflict_count
- conflicts
- error message，如果失败

`result.html` 用于三维可视化检查。

---

## 6. 输入 JSON 格式

### workspace 示例

```json
{
  "min": [0, 0, 0],
  "max": [1000, 800, 600],
  "resolution": 20
}
```

### obstacle 示例

第一阶段只支持 box 障碍物。

```json
{
  "id": "engine",
  "type": "box",
  "min": [300, 200, 100],
  "max": [500, 500, 400]
}
```

### pipe 示例

```json
{
  "id": "pipe_1",
  "start": [50, 100, 100],
  "end": [900, 600, 300],
  "diameter": 20,
  "clearance": 15,
  "min_bend_radius": 60
}
```

### clamp candidate 示例

```json
{
  "id": "clamp_1",
  "position": [300, 300, 200],
  "radius": 80
}
```

---

## 7. 推荐目录结构

请按下面结构组织项目：

```text
pipe_routing/
  __init__.py
  io.py
  grid.py
  astar3d.py
  collision.py
  pipe_rules.py
  multi_pipe.py
  visualize.py

data/
  demo_case.json

outputs/
  result.json
  result.html

tests/
  test_grid.py
  test_astar3d.py
  test_collision.py
  test_multi_pipe.py

run_demo.py
requirements.txt
README.md
```

---

## 8. 各模块职责

### pipe_routing/io.py

负责：

- 读取 JSON 输入
- 定义 dataclass：
  - Workspace
  - Obstacle
  - Pipe
  - ClampCandidate
  - RoutingCase
- 做基础字段校验
- 报错信息要清楚，方便定位输入问题

---

### pipe_routing/grid.py

负责：

- 根据 workspace 创建 3D grid
- world 坐标转 grid index
- grid index 转 world 坐标
- 判断 index 是否在 workspace 范围内
- 判断 cell 是否被 box obstacle 占据
- 支持 obstacle inflate

障碍物膨胀规则：

```text
inflate_distance = pipe.diameter / 2 + pipe.clearance
```

含义：

管路不是一条没有宽度的线，而是有直径的实体。为了保证管子不碰障碍物，需要把障碍物向外膨胀一段距离，再用管路中心线去寻路。

---

### pipe_routing/astar3d.py

负责：

- 单根管路 3D A* 搜索
- 使用 26 邻域
- heuristic 使用欧氏距离
- cost 至少包含：
  - 路径长度
  - 转弯惩罚
- 后续可以扩展：
  - 靠近固定点奖励
  - 远离障碍物奖励
  - 弯曲半径惩罚
- 输出 world 坐标路径
- 如果找不到路径，返回 success = false，不要让程序崩溃

---

### pipe_routing/pipe_rules.py

负责：

- 计算路径长度
- 计算 bend_count
- 简单检查转角是否过急
- 简单检查最小弯曲半径或转角违规
- 后续扩展制造规则，例如：
  - 最小直线段
  - 固定点附近规则
  - 更真实的弯曲半径
  - 可制造性评分

---

### pipe_routing/collision.py

负责：

- 计算点到点距离
- 计算线段到线段最小距离
- 检查管路之间是否冲突
- 输出冲突列表

管路冲突判断：

```text
distance(segment_1, segment_2)
<
pipe_1.diameter / 2 + pipe_2.diameter / 2 + max(pipe_1.clearance, pipe_2.clearance)
```

如果满足上面条件，就认为两根管路距离过近。

冲突结果中建议包含：

- pipe_a
- pipe_b
- segment_a_index
- segment_b_index
- distance
- required_distance
- conflict_point，可选

---

### pipe_routing/multi_pipe.py

负责：

- 多根管子按顺序规划
- 每规划完一根管子，把它作为动态障碍物
- 后续管子需要避开已规划管子
- 最后统一做管-管冲突检测
- 输出每根管子的规划结果

当前阶段不要求完整 CBS，只做简单可运行版本：

```text
先规划 pipe_1
再把 pipe_1 当作动态障碍物
再规划 pipe_2
再把 pipe_1 + pipe_2 当作动态障碍物
再规划 pipe_3
...
```

动态障碍物简化处理方式：

- 可以先把已规划路径附近的 grid cell 标记为占据
- 膨胀半径按已规划管子的 diameter / 2 + clearance 计算
- 后续管路避开这些 cell

---

### pipe_routing/visualize.py

负责：

- 使用 plotly 生成：

```text
outputs/result.html
```

可视化要求：

- 障碍物：灰色半透明 box
- 每根管路：不同颜色折线
- 起点：标出来
- 终点：标出来
- 候选固定点：点或球
- 冲突点：红色点
- 图中要能看出三维空间布局

---

### run_demo.py

负责：

1. 读取 `data/demo_case.json`
2. 执行多管规划
3. 生成 `outputs/result.json`
4. 生成 `outputs/result.html`
5. 在终端打印摘要，例如：

```text
pipe_1 success=True length=1234.5 bend_count=8 conflict_count=0
pipe_2 success=True length=1180.0 bend_count=10 conflict_count=0
pipe_3 success=False error="No path found"
```

---

## 9. 测试要求

使用 pytest。

至少包含以下测试。

### tests/test_grid.py

测试：

- world_to_index
- index_to_world
- obstacle inflate
- box 占据判断
- 越界 index 判断

### tests/test_astar3d.py

测试：

- 单根管路可以从 start 到 end
- 遇到 box 障碍物时可以绕开
- 找不到路径时能返回失败状态，而不是崩溃

### tests/test_collision.py

测试：

- 两根管路距离足够远时无冲突
- 两根管路距离过近时能检测到冲突
- 线段到线段距离计算结果合理

### tests/test_multi_pipe.py

测试：

- demo_case 至少能成功规划 1 根以上管路
- 输出结果中包含 path、length、bend_count、success
- 多管规划失败时不会导致整个程序崩溃

---

## 10. requirements.txt

至少包含：

```text
numpy
plotly
pytest
```

不要加入没有必要的大型依赖。

第一阶段不要引入：

- open3d
- trimesh
- pyvista
- scipy
- ROS 相关依赖
- CATIA 相关依赖

后续确实需要时再加。

---

## 11. README.md 要包含

README 中请写清楚：

1. 项目目标
2. 当前 MVP 功能
3. Windows + conda 环境创建方式
4. 安装依赖命令
5. 运行 demo 命令
6. 运行测试命令
7. 输入 JSON 字段说明
8. 输出 result.json 字段说明
9. 当前限制
10. 后续可扩展方向

Windows + conda 示例命令：

```bash
conda create -n pipe-routing python=3.10 -y
conda activate pipe-routing
pip install -r requirements.txt
python run_demo.py
pytest -q
```

打开可视化结果：

```powershell
start outputs\result.html
```

---

## 12. 编码要求

请遵守：

1. 代码清晰。
2. 函数不要太长。
3. 使用类型注解。
4. 尽量使用 dataclass。
5. 关键逻辑写 docstring 或注释。
6. 不要硬编码绝对路径。
7. 所有路径使用 pathlib。
8. 保证 Windows 下可运行。
9. 出错时给出清楚错误信息。
10. 不要吞掉异常。
11. 不要把测试写成没有意义的“永远通过”。
12. 修改代码后请尽量运行：
    - `python run_demo.py`
    - `pytest -q`

---

## 13. 当前不要做的事情

当前阶段不要做：

- CATIA 插件
- STP / STEP 解析
- OBJ / STL 解析
- GUI 软件
- Web 服务
- 数据库
- Docker
- ROS
- 真实汽车 CAD 模型导入
- 完整 CBS
- 复杂 BOM 自动同步
- 电磁干扰规则
- 复杂制造工艺数据库

这些功能放到 MVP 跑通之后再做。

---

## 14. 第一版实现目标

第一版只需要实现：

```text
JSON 输入
3D grid
box 障碍物
障碍物膨胀
单管 3D A*
多管顺序规划
动态障碍物
管-管冲突检测
result.json 输出
plotly HTML 可视化
pytest 基础测试
```

第一版不追求工业级最优解，只要求：

- 能运行
- 能看结果
- 代码结构清楚
- 后续能继续扩展

---

## 15. 后续扩展方向

MVP 跑通后，可以按顺序扩展：

1. 候选固定点奖励
2. 最小弯曲半径严格检查
3. 路径平滑
4. 局部重算
5. 简化 CBS 回退重算
6. STP / OBJ 导入
7. CATIA 接口
8. BOM 输出
9. 布线场景扩展
