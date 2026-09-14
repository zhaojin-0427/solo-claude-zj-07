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

### 调校批次

调校批次从一个**不可变方案版本**取数（默认最新版本，可用查询参数 `?version=N`
固定），创建时只填写调校仪器与轮圈影响参数；批次状态依次为
`collecting`（采集中）→ `adjusting`（调整中）→ `finalized`（已定稿）。

| 方法 | 路径 | 说明 |
|---|---|---|
| POST | `/plans/{plan_id}/batches` | 创建调校批次（固定来源方案版本） |
| GET | `/batches` | 列出批次（可按 `?plan_id=` 过滤） |
| GET | `/batches/{batch_id}` | 读取批次当前状态（测点、各轮、锁定、累计转动） |
| POST | `/batches/{batch_id}/measurements` | 沿圈孔角序提交整轮张力计与径向/横向读数 |
| POST | `/batches/{batch_id}/locks` | 锁定/解锁辐条（仅采集中；锁定后不得调整） |
| GET | `/batches/{batch_id}/proposals` | 步进候选方案与逐步预测（按字典序排序） |
| POST | `/batches/{batch_id}/rounds/confirm` | 确认候选，冻结本轮测量/动作/结果，进入调整中 |
| POST | `/batches/{batch_id}/rounds/complete` | 完成本轮，下一轮从该快照继续 |
| POST | `/batches/{batch_id}/rounds/cancel` | 放弃调整，回到采集中（冻结轮保留在轨迹） |
| POST | `/batches/{batch_id}/finalize` | 定稿（保留来源方案版本与完整调校轨迹） |
| GET | `/batches/{batch_id}/trajectory` | 完整调校轨迹与只增事件流 |

创建批次填写：张力计校准曲线（`reading → tension_n`，≥2 点、读数严格递增、
张力单调不减，相邻点之间线性插值）、百分表径向/横向零位（测点读数减去零位
为相对跳动）、辐条螺纹螺距、轮圈径向与横向影响系数、张紧传递系数 τ、
全局/左右侧张力上下限、径向/横向超限门限与单孔每轮转动上限。

每轮沿**圈孔角序**提交全部辐条的 `(gauge_reading, radial_mm, lateral_mm)`，
允许从任意孔起步（循环移位）但不得逆序。服务端换算实际张力，并结合快照中的
孔位映射与实际角度计算：偏心（径向一阶谐波，幅值与偏心角）、碟形偏移
（横向加权均值，正=偏右/驱动侧）、左右侧张力均值与**局部张力离散度**
（同侧角序 5 孔循环滑动窗标准差）。缺测、孔号重复、顺序错误、读数越出校准
范围、孔号不属于来源方案，均返回具体测点（孔号/序号）；已定稿批次追加任何
数据一律拒绝。

`GET /proposals` 返回候选列表：候选 0 恒为**不动作基线**（可直接确认以只
冻结测量），其余候选按 **超限测点数 → 最大跳动 → 局部张力离散度 → 总转动量**
字典序升序排列，且必须严格优于基线才输出。每个候选给出：

- 逐步动作：自起始角沿角序、左右就近配对交错，按"层"铺放
  （先 1/2 圈、再 1/4、最后 1/8，每孔每层至多一步，避免先把单孔深拧到位），
  逐步预测执行后的超限量、最大跳动、左右张力与累计转动量；
- `spoke_actions`：各孔汇总的收紧/拧松圈数与张力前后值；
- `blocked_spokes`：被锁定（`locked`）或已在张力上/下限（
  `at_upper_limit_no_tighten` / `at_lower_limit_no_loosen`）而不可调整的孔。

确认一轮后该轮的测量、候选与选中结果、当时的锁定状态全部冻结；下一轮从
快照继续，累计转动量按孔保留。定稿后的 `trajectory` 记录来源 `plan_id` +
`version` + 公式版本（方案/调校/校准三个）、逐轮测量指标、动作、逐步预测与
累计转动量；存储层另有 `batch_events` 只增事件流（创建/采集/候选缓存/
锁定/确认/完成/取消/定稿），可重放完整调校过程。

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

### 调校模型（formula_version: wheel-tuning/1.0）

```
T       = 张力计校准曲线线性插值（越出标定点范围拒绝）
ΔL      = u·P（u=条帽转动圈数，P=螺距；收紧 u>0，拧松 u<0）
ΔT      = τ·A·E·P/L·u  （τ=张紧传递系数，A=πd²/4，E=205900 N/mm²）
Δr_i    = −k_radial · Σ_j w(θ_i−θ_j)·u_j
Δz_i    = k_lateral · (右侧 u 取正、左侧 u 取负) · Σ_j w(θ_i−θ_j)·u_j
w(x)    = (1 + cos(πx/2))/2，x=孔间角/平均节距，|x| ≤ 2（钟形窗）
偏心    = 径向读数（去恒定半径基线）一阶谐波幅值 hypot(Σwr cosθ, Σwr sinθ)
碟形偏移 = Σ w_i·z_i（横向读数相对零位，正 = 偏右/驱动侧）
离散度  = 同侧角序 5 孔循环滑动窗张力标准差（各侧窗均值/最大值）
候选排序 = 超限测点数 ↑ → 最大跳动 hypot(径残差, 横残差) ↑ → 离散度 ↑ → 总转动量 ↑
```

τ 表示条帽拉入位移中由辐条弹性承担的比例（其余由轮圈弯曲吸收，
典型 0.3~0.7）；搜索先以平滑目标引导越门限与多孔协调（碟形/偏心），
再按上述官方字典序精修，最终只输出严格优于不动作基线的候选。


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
| `PLAN_NOT_FOUND` / `BATCH_NOT_FOUND` / `VERSION_NOT_FOUND` / `CANDIDATE_NOT_FOUND` | 方案/批次/版本/候选不存在（404 或 400） |
| `CALIBRATION_INVALID` | 张力计校准曲线点数不足、读数不严格递增或张力非单调 |
| `MEASUREMENT_MISSING_HOLE` | 缺测，`details.missing_rim_holes` 给出缺测孔号 |
| `MEASUREMENT_DUPLICATE_HOLE` | 孔号重复，给出孔号与两个测点序号 |
| `MEASUREMENT_HOLE_UNKNOWN` | 测点孔号不属于来源方案 |
| `MEASUREMENT_ORDER_INVALID` | 未沿圈孔角序提交，给出序号、孔号与期望的下一孔号 |
| `MEASUREMENT_OUT_OF_RANGE` | 张力计读数越出校准范围，给出测点与校准范围 |
| `BATCH_STATUS_INVALID` | 状态机拒绝（如调整中采集/锁定、定稿后追加数据） |
| `BATCH_NO_MEASUREMENT` | 未采集本轮测量就请求方案/确认 |
| `BATCH_FINALIZE_EMPTY` | 没有任何已确认调校轮就定稿 |
| `BATCH_HOLE_UNKNOWN` | 锁定/解锁孔号不属于来源方案 |

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
