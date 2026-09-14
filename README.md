# Wheel Lacing API — 自行车轮组编轮计算本机服务

正式编轮前，根据轮圈 / 花鼓 / 辐条规格计算可实际装配的辐条长度与穿法，
并结合库存辐条与垫圈给出合格组合。所有方案以**不可变版本**存入 SQLite，
同一版本重复读取结果逐字节一致。

## 运行

```bash
pip install -r requirements.txt
python3 -m uvicorn wheel_api.app:app --port 8000
# 或： python3 -m wheel_api.app
```

数据库默认写入 `./wheel_plans.db`，可用环境变量 `WHEEL_API_DB` 指定路径。
交互式文档：http://127.0.0.1:8000/docs

## 接口

| 方法 | 路径 | 说明 |
|---|---|---|
| POST | `/plans` | 新建方案（版本 1：几何 + 穿法 + SVG） |
| GET | `/plans` | 列出全部方案 |
| GET | `/plans/{plan_id}` | 读取最新版本快照 |
| GET | `/plans/{plan_id}/versions/{v}` | 读取指定版本快照（不可变） |
| POST | `/plans/{plan_id}/versions` | 以新的轮组输入生成新版本 |
| POST | `/plans/{plan_id}/optimize` | 提交库存/垫圈/张力约束，生成含优化结果的新版本 |

### 输入约定

- **圈孔编号** `0..N-1`，从驱动侧看顺时针递增；偶数孔在右侧，奇数孔在左侧。
- **非等距钻孔** `rim.hole_table`：逐孔给出 `id`（唯一编号）、`angle_deg`（圆周角）、
  `side`（left/right）、`axial_offset_mm`（轴向偏移），支持成对辐条孔；提供时覆盖
  等距模式与 `hole_offset_*`，条数须等于 `holes`，左右侧孔数须相等且各为偶数。
- **阀孔位置** `valve_position`：阀孔位于该编号圈孔与角序下一圈孔之间。
- **法兰孔角度**：默认右法兰 0° 起、左法兰错开半个节距；可用
  `hub.flange_phase_left/right_deg` 设置起始相位，或用
  `hub.flange_angles_left/right_deg` 逐孔给出角度（两者互斥，条数 = 每法兰孔数）。
- **法兰孔编号**：右法兰孔 `j` 与圈孔 `2j` 同相；左法兰孔与右法兰错开半个节距（标准花鼓结构）。
- **内外穿** `heads_in`：指定哪一组（`leading` 顺向 / `trailing` 逆向）辐条头朝法兰内侧穿入。
- **阀孔净空** `valve_clearance_min_mm`：自动穿法搜索时须满足的净空下限（默认 0，即仅取最优）。

### 创建方案示例

```bash
curl -X POST http://127.0.0.1:8000/plans -H 'Content-Type: application/json' -d '{
  "name": "road-rear",
  "rim":  {"erd_mm": 600.0, "holes": 36,
           "hole_offset_left_mm": 1.5, "hole_offset_right_mm": 1.5,
           "valve_position": 35},
  "hub":  {"holes_per_flange": 18,
           "flange_pcd_left_mm": 58.0, "flange_pcd_right_mm": 45.0,
           "center_to_flange_left_mm": 35.0, "center_to_flange_right_mm": 20.0,
           "spoke_hole_diameter_mm": 2.4},
  "left":  {"cross": 3, "heads_in": "trailing"},
  "right": {"cross": 3, "heads_in": "trailing"},
  "spoke_diameter_mm": 2.0
}'
```

响应快照包含：

- `geometry.angle_source`：角度来源（轮圈 `uniform`/`hole_table`，花鼓
  `default_zero`/`default_half_pitch`/`phase`/`explicit`）；
- `geometry.sides`：每侧名义辐条长度（含内/外穿修正）、交叉角、有效法兰偏距、张角；
- `geometry.per_hole`：**逐孔**实际角度、辐条长度、入圈角、法兰出线角、张角；
- `geometry.tension_ratio_left_to_right`：左右张力比；
- `lacing.first_spoke`：避开阀孔的首根定位——取阀孔角序下一圈孔，
  相位自动二选一使阀孔净空最大；
- `lacing.flange_shift`：解等价于整体循环移位时给出每侧法兰对齐量，
  非循环解的侧为 `null`；
- `lacing.valve`：阀孔位置、实际净空与净空下限；
- `lacing.sequence`：编轮次序（首根所在组为第 1 步，其余按 内穿组 → 外穿组、
  首根侧 → 另一侧，组内从紧邻阀孔的圈孔起按实际角度的旋转顺序排列）；
- `lacing.mapping`：完整孔位映射；
- `svg`：左右双视图 SVG 穿线图（标注孔号、阀孔、首根、顺/逆向与内/外穿）；
- `formulas` / `formula_version`：本版本使用的全部计算公式。

自动穿法按**实际孔位**回溯搜索全部合法孔位双射（不限于整体循环移位）：
同向辐条的弦不得相交、每根辐条与反向辐条的实际弦交叉数等于设置值、
顺/逆向数量平衡，且阀孔净空满足设置；在合法双射中取净空最大者。
无解时返回 `LACING_INFEASIBLE` 及冲突孔（孔号、侧别、原因）。

### 库存组合优化示例

```bash
curl -X POST http://127.0.0.1:8000/plans/whl_xxx/optimize -H 'Content-Type: application/json' -d '{
  "inventory": [{"length_mm": 288.0}, {"length_mm": 289.0, "count": 24}, {"length_mm": 290.0}],
  "washers_mm": [0.5, 1.0],
  "tension_min_n": 600.0, "tension_max_n": 1400.0,
  "length_tolerance_mm": 1.0,
  "limit": 10
}'
```

装配关系：垫圈厚度 `t` 使条帽座外移，等效理想长度变为 `ideal + t`。
偏差按**逐孔**理想长度判定（同侧内/外穿修正使各孔理想长度存在散布）：
`err_hole = 库存条长 − t − ideal_hole`。合格条件（对该侧全部孔取最不利值）：

- `max|err_hole| ≤ length_tolerance_mm`（允许长度误差，按逐孔最大偏差判定与报告）；
- `螺纹啮合 = spoke_thread_length + min(0, min err_hole) ≥ min_thread_engagement_mm`；
- `max err_hole ≤ max_protrusion_mm`（条帽顶端外露）；
- 张力：`T_left = ratio × T_right`，两侧均须落在张力上下限内（在可行区间取中点使余量最大）；
- 库存数量：同一长度的多行库存先合并（数量累加，任一行不限量则该长度不限量），
  再与两侧各 n 根的需求量比较。

组合按 **最大长度偏差升序 → 张力余量降序 → 规格种数升序** 排序，
`excluded` 中给出各原因被淘汰的候选数；每个组合附带
`deviation_range_mm`（该侧逐孔偏差区间）供核查。

## 计算公式（formula_version: wheel-geometry/1.2）

```
L      = sqrt(R² + r_eff² + w_eff² − 2·R·r_eff·cos δ) − s/2
δ      = 实际孔位夹角（角度来源见 angle_source；标准等距穿法 = 2π·cross/n，n = 每侧孔数）
r_eff  = PCD/2 ∓ d/2（内穿 −，外穿 +；d = 辐条杆径）
w_eff  = center_to_flange − rim_hole_offset（同侧圈孔横向偏移，逐孔取值）
张角    = asin(w_eff / (L + s/2))
张力比  = T_left / T_right = sin(张角_right) / sin(张角_left)
入圈角  = 辐条方向与圈孔处半径方向的夹角（0 = 正对轴心）
出线角  = 辐条轮平面投影与法兰孔切线的夹角（0 = 相切，90 = 径向）
```

## 错误返回

领域错误返回 400（未找到为 404），结构为
`{"error": {"code", "message", "details"}}`，并给出具体孔号与原因：

| code | 含义 |
|---|---|
| `HOLE_COUNT_INVALID` | 圈孔数不能被 4 整除，或孔表左右侧孔数不等/为奇（每侧需偶数孔均分顺/逆向） |
| `HOLE_COUNT_MISMATCH` | 每侧圈孔数 ≠ 每法兰孔数 |
| `VALVE_POSITION_INVALID` | 阀孔位置不是有效的圈孔编号 |
| `CROSS_INFEASIBLE` | 交叉数超过每侧孔数允许的上限 `(n−2)//4` |
| `LACING_INFEASIBLE` | 自动穿法按实际孔位搜索无解（交叉方向/交叉数冲突或阀孔净空不足），`details.conflicts` 给出冲突孔 |
| `DUPLICATE_HOLE_MAPPING` | 法兰孔被重复占用（含孔号与映射序号）；圈孔重复占用在 Pydantic 阶段返回 422 |
| `MAPPING_INCOMPLETE` | 自定义映射未覆盖全部孔位 |
| `MAPPING_INVALID` | 自定义映射孔号不存在/越界或圈孔侧别不符 |
| `GEOMETRY_INVALID` | 几何参数矛盾（如有效偏距 ≤ 0） |

孔表覆盖/编号重复、圆周角重复、侧别数量、法兰相位与逐孔角度互斥及覆盖、
`mapping_override` 圈孔重复占用等输入校验由 Pydantic 完成，返回 422 及具体字段位置。

## 不可变性

- 快照以键排序的规范化 JSON 整体写入 `versions` 表，只增不改；
- 每次 `POST /plans/{id}/versions` 或 `/optimize` 生成新版本号，旧版本内容不变；
- 快照保留全部输入、孔位映射、逐孔结果与计算公式文本，可脱离代码追溯；
- 方案与首个版本原子写入：创建请求校验/计算失败（400/422）时不产生任何
  方案记录，不会留下可列出却没有快照的空方案。

## 测试

```bash
python3 -m pytest tests/ -q
```

覆盖：几何基准值（径向/对称）、张力比、孔数不配、交叉不可行、
自定义映射圈孔重复（422）与法兰孔重复、阀孔避让与首根定位、编轮次序完整性、
优化排序与排除（长度误差/螺纹啮合/张力窗口）、版本不可变重复读取、
非等距孔表与成对辐条（实际角度逐孔结果、搜索确定性）、法兰相位/逐孔角度、
孔表与法兰角度校验（422）、自动穿法无解冲突孔、既有快照兼容读取、
12 孔非等距非循环合法解搜索、创建失败不写方案记录。
