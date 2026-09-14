"""调校领域：张力换算、跳动/偏心/碟形/离散度分析、影响矩阵与步进方案搜索。

数据全部来自创建批次时锁定的不可变方案版本快照（逐孔角度、侧别、辐条长度），
批次自身只追加测量与调校轨迹。

力学模型（经验线性模型，系数在创建批次时由用户标定/填写）：
- 张力：张力计读数沿校准曲线线性插值换算实际张力 N；
- 条帽转动：螺距 P（mm/圈），收紧 u 圈 -> 条帽拉入 u*P，辐条弹性伸长
  ΔL = u*P/L，ΔT = τ·A·E·P/L·u（τ = 张紧传递系数：条帽拉入位移中
  由辐条弹性承担的比例，其余由轮圈弯曲/横向变形吸收；典型 0.3~0.7）；
- 跳动影响（钟形窗，半宽 2 个节距）：
  径向：收紧辐条把圈孔处半径拉小，峰值 −k_radial（mm/圈）；
  横向：右侧收紧把轮圈拉向右侧 +k_lateral，左侧收紧拉向左侧 −k_lateral；
  w(x) = (1 + cos(π x / 2))/2，x = 孔间角 / 平均节距，|x| ≤ 2；
- 偏心：径向读数的一阶谐波（加权最小二乘），偏心量与偏心角；
- 碟形偏移：横向读数的加权均值（正 = 偏右/驱动侧）；
- 局部张力离散度：同侧按角序取 5 孔滑动窗（循环），窗内张力标准差，
  汇总各侧窗标准差的均值与最大值。

非等距/成对孔位一律使用快照中的实际角度与角序权重，不假设等距。
"""

from __future__ import annotations

import math
from bisect import bisect_left

from .errors import WheelError
from .geometry import r3

TUNING_FORMULA_VERSION = "wheel-tuning/1.0"
CALIBRATION_FORMULA_VERSION = "tension-calibration/1.0"

# 钢辐条弹性模量 N/mm²
STEEL_E_N_PER_MM2 = 205_900.0

# 影响钟形窗半宽（以平均孔距为单位）
BELL_HALF_WIDTH = 2.0

# 同侧局部张力离散窗（孔）
LOCAL_WINDOW = 5

TUNING_FORMULAS = {
    "tension": "T = 张力计校准曲线线性插值（相邻标定点之间），读数越出标定点范围拒绝",
    "nipple_turn": "ΔL = u·P（u=条帽转动圈数，P=螺距 mm/圈）；收紧 u>0，拧松 u<0",
    "tension_delta": "ΔT = τ·A·E·P/L·u，τ=张紧传递系数，A = π·d²/4（d=辐条杆径），E=205900 N/mm²（钢），L=该孔实际辐条长度",
    "radial_influence": "Δr_i = −k_radial·Σ_j w(θ_i−θ_j)·u_j（收紧把半径拉小）",
    "lateral_influence": "Δz_i = k_lateral·(右侧 u 取正、左侧 u 取负)·Σ_j w(θ_i−θ_j)·u_j",
    "bell_window": "w(x) = (1 + cos(π x / 2)) / 2，x = 孔间角/平均节距，|x| ≤ 2，否则 0",
    "eccentricity": "加权一阶谐波：e_x=Σw_i·r_i·cosθ_i, e_y=Σw_i·r_i·sinθ_i；偏心量=hypot(e_x,e_y)",
    "dish_offset": "碟形偏移 = Σw_i·z_i（横向读数相对零位，正 = 偏右/驱动侧）",
    "local_dispersion": "同侧角序 5 孔循环滑动窗内张力标准差，取各侧窗均值/最大值",
    "step_granularity": "条帽步进以 1/8 圈（45°）为最小单位；编排按层铺放（先 1/2=4 单位、再 1/4=2、最后 1/8=1），层内左右就近配对交错，逐步预测",
    "search": "阶段一平滑目标（超限平方+残差平方+碟形/偏心全局项）引导越门限与多孔协调；阶段二按官方字典序精修（含持平行走），只输出严格优于不动作基线的候选",
    "candidate_ranking": "候选依次比较：超限测点数↑ → 最大跳动 hypot(径,横)↑ → 局部张力离散度↑ → 总转动量↑（全部取小）",
}


# ---------------------------------------------------------------------------
# 校准曲线
# ---------------------------------------------------------------------------

def validate_curve(points: list[tuple[float, float]]) -> list[tuple[float, float]]:
    """校验校准曲线：≥2 点、读数严格递增、张力单调不减。"""
    if len(points) < 2:
        raise WheelError(
            "CALIBRATION_INVALID",
            "张力计校准曲线至少需要 2 个标定点",
            {"points": len(points)},
        )
    ordered = sorted(points, key=lambda p: p[0])
    for (x0, t0), (x1, t1) in zip(ordered, ordered[1:]):
        if x1 <= x0:
            raise WheelError(
                "CALIBRATION_INVALID",
                f"校准曲线读数必须严格递增（{x0} 与 {x1} 重复或逆序）",
                {"reading_a": x0, "reading_b": x1},
            )
        if t1 + 1e-9 < t0:
            raise WheelError(
                "CALIBRATION_INVALID",
                f"校准曲线张力必须单调不减（读数 {x0}->{x1}，张力 {t0}->{t1}）",
                {"reading_a": x0, "reading_b": x1, "tension_a_n": t0, "tension_b_n": t1},
            )
    return ordered


def reading_range(points: list[tuple[float, float]]) -> tuple[float, float]:
    return points[0][0], points[-1][0]


def tension_from_reading(reading: float, points: list[tuple[float, float]]) -> float:
    """校准曲线分段线性插值；越界返回 None（由调用方按测点报错）。"""
    lo, hi = points[0][0], points[-1][0]
    if reading < lo or reading > hi:
        return None
    xs = [p[0] for p in points]
    k = bisect_left(xs, reading)
    if k < len(points) and points[k][0] == reading:
        return points[k][1]
    x0, t0 = points[k - 1]
    x1, t1 = points[k]
    return t0 + (t1 - t0) * (reading - x0) / (x1 - x0)


# ---------------------------------------------------------------------------
# 方案快照取数
# ---------------------------------------------------------------------------

def extract_spokes(plan_snapshot: dict) -> list[dict]:
    """从不可变方案版本快照提取逐孔 {rim_hole, side, angle_rad, length_mm}。"""
    diameter = float(plan_snapshot["geometry"]["spoke_diameter_mm"])
    out = []
    for h in plan_snapshot["geometry"]["per_hole"]:
        out.append({
            "rim_hole": h["rim_hole"],
            "side": h["side"],
            "angle_rad": math.radians(h["rim_angle_deg"]),
            "length_mm": h["length_mm"],
            "spoke_diameter_mm": diameter,
        })
    out.sort(key=lambda s: (s["angle_rad"], s["rim_hole"]))
    return out


def _gap_weights(angles_rad: list[float]) -> list[float]:
    """非等距角序权重：每孔取与相邻两孔夹角一半之和，归一化到合计 1。"""
    n = len(angles_rad)
    gaps = []
    for i in range(n):
        a_prev = angles_rad[i - 1]
        a_next = angles_rad[(i + 1) % n]
        a = angles_rad[i]
        g_lo = (a - a_prev) % (2.0 * math.pi)
        g_hi = (a_next - a) % (2.0 * math.pi)
        gaps.append((g_lo + g_hi) / 2.0)
    total = sum(gaps)
    return [g / total for g in gaps]


def _bell(x: float) -> float:
    if abs(x) >= BELL_HALF_WIDTH:
        return 0.0
    return 0.5 * (1.0 + math.cos(math.pi * x / BELL_HALF_WIDTH))


def build_influence(spokes: list[dict], pitch_rad: float,
                    k_radial: float, k_lateral: float,
                    thread_pitch_mm: float, tension_transfer: float = 1.0) -> dict:
    """影响矩阵：rad/ lat 为 N×N（行=观测孔 i，列=动作孔 j，单位 mm/圈）。

    同时预计算每根动作辐条 j 的钟形窗邻接列 cols[j]，贪心搜索只需更新窗内孔。
    张力刚度乘以张紧传递系数 τ（条帽拉入位移中由辐条弹性承担的比例）。
    """
    n = len(spokes)
    rad = [[0.0] * n for _ in range(n)]
    lat = [[0.0] * n for _ in range(n)]
    cols = [[] for _ in range(n)]
    for j, sj in enumerate(spokes):
        for i, si in enumerate(spokes):
            d = math.atan2(
                math.sin(si["angle_rad"] - sj["angle_rad"]),
                math.cos(si["angle_rad"] - sj["angle_rad"]),
            )
            w = _bell(d / pitch_rad)
            if w == 0.0:
                continue
            rad[i][j] = -k_radial * w          # 收紧 -> 半径减小
            sign_z = 1.0 if sj["side"] == "right" else -1.0
            lat[i][j] = sign_z * k_lateral * w
            cols[j].append(i)
    stiffness = []
    for s in spokes:
        area = math.pi * (s["spoke_diameter_mm"] ** 2) / 4.0
        stiffness.append(tension_transfer
                          * area * STEEL_E_N_PER_MM2 * thread_pitch_mm / s["length_mm"])
    return {"radial": rad, "lateral": lat, "columns": cols,
            "stiffness_n_per_turn": stiffness}


# ---------------------------------------------------------------------------
# 统计与分析
# ---------------------------------------------------------------------------

def _std(values: list[float]) -> float:
    if not values:
        return 0.0
    m = sum(values) / len(values)
    return math.sqrt(sum((v - m) ** 2 for v in values) / len(values))


def _local_dispersion(ordered_values: list[float]) -> tuple[float, float]:
    """5 孔循环滑动窗标准差，返回 (窗均值, 窗最大值)。"""
    n = len(ordered_values)
    half = LOCAL_WINDOW // 2
    if n <= LOCAL_WINDOW:
        s = _std(ordered_values)
        return s, s
    wins = []
    for i in range(n):
        window = [ordered_values[(i + d) % n] for d in range(-half, half + 1)]
        wins.append(_std(window))
    return sum(wins) / len(wins), max(wins)


def summarize(name_idx: list[dict], weights: list[float], radial: list[float],
              lateral: list[float], tension: list[float], limits: dict,
              include_violations: bool = True) -> dict:
    """对一组（可能为预测后的）整圈状态计算全部指标。

    name_idx: 角序排列的 [{rim_hole, side, angle_rad}]，与各数组同序。
    limits: {radial_tolerance_mm, lateral_tolerance_mm,
             tension_min_n:[逐孔], tension_max_n:[逐孔]}
    """
    angles = [s["angle_rad"] for s in name_idx]
    n = len(name_idx)

    # 径向：百分表只反映半径变化，去除恒定基线（均值），保留偏心一阶谐波；
    # 横向：零位相对台架/轮组目标中心设定，碟形偏移直接留在残差中参与超限与峰值。
    mean_r = sum(w * v for w, v in zip(weights, radial))
    mean_l = sum(w * v for w, v in zip(weights, lateral))
    e_x = sum(w * (r - mean_r) * math.cos(a) for w, r, a in zip(weights, radial, angles))
    e_y = sum(w * (r - mean_r) * math.sin(a) for w, r, a in zip(weights, radial, angles))
    ecc = math.hypot(e_x, e_y)
    ecc_angle = math.degrees(math.atan2(e_y, e_x)) % 360.0

    res_r = [r - mean_r for r in radial]
    res_l = list(lateral)

    side_stats = {}
    for side in ("left", "right"):
        idx = [i for i, s in enumerate(name_idx) if s["side"] == side]
        vals = [tension[i] for i in idx]
        local_mean, local_max = _local_dispersion(vals)
        side_stats[side] = {
            "mean_n": r3(sum(vals) / len(vals)),
            "min_n": r3(min(vals)),
            "max_n": r3(max(vals)),
            "std_n": r3(_std(vals)),
            "local_std_mean_n": r3(local_mean),
            "local_std_max_n": r3(local_max),
            "cv": r3(_std(vals) / (sum(vals) / len(vals))) if vals and sum(vals) > 0 else None,
        }
    dispersion = max(side_stats["left"]["local_std_mean_n"],
                     side_stats["right"]["local_std_mean_n"])

    combined = [math.hypot(rr, ll) for rr, ll in zip(res_r, res_l)]
    peak_i = max(range(n), key=lambda i: combined[i])

    violations = []
    for i, s in enumerate(name_idx):
        if abs(res_r[i]) > limits["radial_tolerance_mm"] + 1e-9:
            violations.append({"rim_hole": s["rim_hole"], "side": s["side"],
                               "type": "radial_runout", "value_mm": r3(res_r[i]),
                               "limit_mm": r3(limits["radial_tolerance_mm"])})
        if abs(res_l[i]) > limits["lateral_tolerance_mm"] + 1e-9:
            violations.append({"rim_hole": s["rim_hole"], "side": s["side"],
                               "type": "lateral_runout", "value_mm": r3(res_l[i]),
                               "limit_mm": r3(limits["lateral_tolerance_mm"])})
        if tension[i] > limits["tension_max_n"][i] + 1e-9:
            violations.append({"rim_hole": s["rim_hole"], "side": s["side"],
                               "type": "tension_over_max", "value_n": r3(tension[i]),
                               "limit_n": r3(limits["tension_max_n"][i])})
        if tension[i] < limits["tension_min_n"][i] - 1e-9:
            violations.append({"rim_hole": s["rim_hole"], "side": s["side"],
                               "type": "tension_under_min", "value_n": r3(tension[i]),
                               "limit_n": r3(limits["tension_min_n"][i])})

    metrics = {
        "violation_count": len(violations),
        "max_runout_mm": r3(combined[peak_i]),
        "max_runout_point": name_idx[peak_i]["rim_hole"],
        "radial_peak_mm": r3(max(abs(v) for v in res_r)),
        "lateral_peak_mm": r3(max(abs(v) for v in res_l)),
        "radial_mean_mm": r3(mean_r),
        "eccentricity_mm": r3(ecc),
        "eccentricity_angle_deg": r3(ecc_angle),
        "dish_offset_mm": r3(mean_l),
        "local_tension_dispersion_n": r3(dispersion),
        "total_turns": 0.0,
    }
    result = {"metrics": metrics, "tension_by_side": side_stats}
    if include_violations:
        result["violations"] = violations
    return result


def metrics_key(summary: dict, total_turns: float) -> tuple:
    m = summary["metrics"]
    return (m["violation_count"], m["max_runout_mm"],
            m["local_tension_dispersion_n"], r3(total_turns))


# ---------------------------------------------------------------------------
# 测量提交校验（错误一律指到测点）
# ---------------------------------------------------------------------------

def validate_readings(readings: list[dict], spokes: list[dict],
                      curve: list[tuple[float, float]]) -> dict:
    """校验测点：未知孔、重复孔、缺测、顺序、校准范围。返回 角序->测点 映射。"""
    by_hole = {s["rim_hole"]: idx for idx, s in enumerate(spokes)}
    angle_of = {s["rim_hole"]: s["angle_rad"] for s in spokes}
    rim_order = [s["rim_hole"] for s in spokes]
    n = len(spokes)
    lo, hi = reading_range(curve)

    seen: dict[int, int] = {}
    by_index: dict[int, dict] = {}
    unknown = []
    out_of_range = []
    for idx, rd in enumerate(readings):
        hole = rd["rim_hole"]
        if hole not in by_hole:
            unknown.append({"index": idx, "rim_hole": hole})
            continue
        if hole in seen:
            raise WheelError(
                "MEASUREMENT_DUPLICATE_HOLE",
                f"孔号 {hole} 的测点重复提交（第 {seen[hole]} 与第 {idx} 个测点）",
                {"rim_hole": hole, "first_index": seen[hole], "duplicate_index": idx},
            )
        seen[hole] = idx
        by_index[idx] = rd
        t = tension_from_reading(rd["gauge_reading"], curve)
        if t is None:
            out_of_range.append({"rim_hole": hole, "index": idx,
                                 "reading": rd["gauge_reading"], "range": [lo, hi]})
    if unknown:
        raise WheelError(
            "MEASUREMENT_HOLE_UNKNOWN",
            f"测点孔号不属于来源方案: {[u['rim_hole'] for u in unknown]}",
            {"points": unknown},
        )
    if out_of_range:
        raise WheelError(
            "MEASUREMENT_OUT_OF_RANGE",
            f"{len(out_of_range)} 个张力计读数越出校准范围 [{lo}, {hi}]",
            {"range": [lo, hi], "points": out_of_range},
        )
    missing = [h for h in rim_order if h not in seen]
    if missing:
        raise WheelError(
            "MEASUREMENT_MISSING_HOLE",
            f"缺测 {len(missing)} 个孔位: {missing[:20]}{' ...' if len(missing) > 20 else ''}",
            {"missing_rim_holes": missing, "expected": n, "submitted": len(seen)},
        )

    # 沿圈孔顺序：允许任意孔起步（循环移位），但方向必须与角序一致、不得回头
    positions = [rim_order.index(rd["rim_hole"]) for rd in readings]
    for k in range(1, len(positions)):
        if (positions[k] - positions[k - 1]) % n != 1:
            raise WheelError(
                "MEASUREMENT_ORDER_INVALID",
                f"测点未沿圈孔顺序提交：第 {k} 个测点（孔 {readings[k]['rim_hole']}）"
                f"未紧跟孔 {readings[k - 1]['rim_hole']} 的角序下一孔",
                {"index": k, "rim_hole": readings[k]["rim_hole"],
                 "previous_rim_hole": readings[k - 1]["rim_hole"],
                 "expected_rim_hole": rim_order[(positions[k - 1] + 1) % n]},
            )
    return {by_hole[rd["rim_hole"]]: rd for rd in readings}


def analyze_measurement(batch: dict, spokes: list[dict], weights: list[float],
                        readings_by_idx: dict[int, dict]) -> dict:
    """换算实际张力并计算偏心、碟形、离散度与超限测点。"""
    curve = [(float(p["reading"]), float(p["tension_n"]))
             for p in batch["calibration_curve"]]
    z0 = batch["radial_zero_mm"]
    l0 = batch["lateral_zero_mm"]

    radial, lateral, tension = [], [], []
    points = []
    for i, s in enumerate(spokes):
        rd = readings_by_idx[i]
        t = tension_from_reading(rd["gauge_reading"], curve)
        r_mm = rd["radial_mm"] - z0
        l_mm = rd["lateral_mm"] - l0
        radial.append(r_mm)
        lateral.append(l_mm)
        tension.append(t)
        points.append({
            "rim_hole": s["rim_hole"],
            "side": s["side"],
            "angle_deg": r3(math.degrees(s["angle_rad"])),
            "gauge_reading": rd["gauge_reading"],
            "tension_n": r3(t),
            "radial_mm": r3(r_mm),
            "lateral_mm": r3(l_mm),
        })

    limits = _limits_arrays(batch, spokes)
    summary = summarize(spokes, weights, radial, lateral, tension, limits)
    return {
        "points": points,
        "metrics": summary["metrics"],
        "tension_by_side": summary["tension_by_side"],
        "violations": summary["violations"],
    }


def _limits_arrays(batch: dict, spokes: list[dict]) -> dict:
    tmin, tmax = [], []
    ov = batch.get("tension_limits_override") or {}
    for s in spokes:
        side = s["side"]
        side_ov = ov.get(side) or {}
        tmin.append(float(side_ov.get("min_n", batch["tension_min_n"])))
        tmax.append(float(side_ov.get("max_n", batch["tension_max_n"])))
    return {
        "radial_tolerance_mm": batch["radial_tolerance_mm"],
        "lateral_tolerance_mm": batch["lateral_tolerance_mm"],
        "tension_min_n": tmin,
        "tension_max_n": tmax,
    }


# ---------------------------------------------------------------------------
# 方案预测：步进候选搜索
# ---------------------------------------------------------------------------

UNIT = 8  # 最小步进 = 1/8 圈，所有转动量用整数单位表示


def _apply_units(radial0, lateral0, tension0, inf, units: list[int]):
    n = len(radial0)
    u = [x / UNIT for x in units]
    radial = [radial0[i] + sum(inf["radial"][i][j] * u[j] for j in range(n))
              for i in range(n)]
    lateral = [lateral0[i] + sum(inf["lateral"][i][j] * u[j] for j in range(n))
               for i in range(n)]
    tension = [tension0[j] + inf["stiffness_n_per_turn"][j] * u[j] for j in range(n)]
    return radial, lateral, tension


def _soft_key(spokes, weights, radial, lateral, tension, limits, units,
              rad_tol, lat_tol):
    """平滑引导目标（仅搜索内部使用）。

    超限平方和 + 全残差平方和（小权重）+ 碟形/偏心全局分量 +
    张力越界重罚 + 微小转动量正则。让单孔步进在尚未把测点压过门限、
    或必须多孔协调（碟形/偏心）时仍有明确下山方向。
    最终方案优劣一律以官方字典序（见 metrics_key）为准。
    """
    n = len(spokes)
    angles = [s["angle_rad"] for s in spokes]
    mean_r = sum(w * v for w, v in zip(weights, radial))
    mean_l = sum(w * v for w, v in zip(weights, lateral))
    e_x = sum(w * (r - mean_r) * math.cos(a) for w, r, a in zip(weights, radial, angles))
    e_y = sum(w * (r - mean_r) * math.sin(a) for w, r, a in zip(weights, radial, angles))
    excess = 0.0
    residual2 = 0.0
    t_pen = 0.0
    for i, s in enumerate(spokes):
        rr = radial[i] - mean_r
        ll = lateral[i]
        er = abs(rr) - rad_tol
        el = abs(ll) - lat_tol
        if er > 0:
            excess += er * er
        if el > 0:
            excess += el * el
        residual2 += rr * rr + ll * ll
        if tension[i] > limits["tension_max_n"][i]:
            d = tension[i] - limits["tension_max_n"][i]
            t_pen += (d / 100.0) ** 2
        if tension[i] < limits["tension_min_n"][i]:
            d = limits["tension_min_n"][i] - tension[i]
            t_pen += (d / 100.0) ** 2
    global_term = 2.0 * mean_l * mean_l + 2.0 * (e_x * e_x + e_y * e_y)
    reg = 1e-6 * sum((u / UNIT) ** 2 for u in units)
    return excess + 0.02 * residual2 + global_term + 100.0 * t_pen + reg


def _search_variant(spokes, weights, radial0, lateral0, tension0, limits, inf,
                    locks, max_units, step_units_set, cap=400):
    """两阶段贪心，输出整轮各孔转动量（1/8 圈整数单位）。

    阶段一（引导）：单孔 + 左右成对动作沿平滑目标下山，使方案能越过
        超限门限并完成碟形/偏心所需的多孔协调；
    阶段二（精修）：从阶段一终点起，只接受使官方字典序
        （超限量→最大跳动→局部张力离散度→总转动量）严格变小或
        前三项持平的动作（少量持平行走以跨越单步平台），记录沿途最佳。
    """
    n = len(spokes)
    units = [0] * n
    locked = set(locks)
    radial, lateral, tension = list(radial0), list(lateral0), list(tension0)
    rad_tol = limits["radial_tolerance_mm"]
    lat_tol = limits["lateral_tolerance_mm"]
    columns = inf["columns"]
    rad_m, lat_m, k_m = inf["radial"], inf["lateral"], inf["stiffness_n_per_turn"]

    local_pairs = set()
    for j in range(n):
        for k in columns[j]:
            if spokes[k]["side"] != spokes[j]["side"]:
                local_pairs.add((min(j, k), max(j, k)))
    by_side = {"left": [], "right": []}
    for idx, s in enumerate(spokes):
        by_side[s["side"]].append(idx)
    for j in by_side["right"]:
        k = min(by_side["left"],
                key=lambda i: abs(math.atan2(
                    math.sin(spokes[i]["angle_rad"] - spokes[j]["angle_rad"]),
                    math.cos(spokes[i]["angle_rad"] - spokes[j]["angle_rad"]))))
        local_pairs.add((min(j, k), max(j, k)))
    pairs = sorted(local_pairs)

    def feasible(j, sign, du_units):
        nu = units[j] + sign * du_units
        if abs(nu) > max_units:
            return False
        t_after = tension[j] + k_m[j] * sign * du_units / UNIT
        if sign > 0 and t_after > limits["tension_max_n"][j] + 1e-9:
            return False
        if sign < 0 and t_after < limits["tension_min_n"][j] - 1e-9:
            return False
        return True

    def all_moves():
        moves = []
        for j in range(n):
            if spokes[j]["rim_hole"] in locked:
                continue
            for step in step_units_set:
                for sign in (1, -1):
                    if feasible(j, sign, step):
                        moves.append(((j, sign * step),))
        for j, k in pairs:
            if (spokes[j]["rim_hole"] in locked or spokes[k]["rim_hole"] in locked):
                continue
            for step in step_units_set:
                for sign in (1, -1):
                    if feasible(j, sign, step) and feasible(k, -sign, step):
                        moves.append(((j, sign * step), (k, -sign * step)))
        return moves

    def probe(move):
        """施加动作，返回 (snapshot)；由 restore 还原。"""
        snap = []
        acted = {}
        for j, dunits in move:
            if j in acted:
                continue
            acted[j] = True
            old_u, old_t = units[j], tension[j]
            touched = {}
            for i in columns[j]:
                touched[i] = (radial[i], lateral[i])
            units[j] = old_u + dunits
            for i in columns[j]:
                radial[i] += rad_m[i][j] * dunits / UNIT
                lateral[i] += lat_m[i][j] * dunits / UNIT
            tension[j] = old_t + k_m[j] * dunits / UNIT
            snap.append((j, old_u, old_t, touched))
        return snap

    def restore(snap):
        for j, old_u, old_t, touched in snap:
            units[j] = old_u
            tension[j] = old_t
            for i, (rv, lv) in touched.items():
                radial[i], lateral[i] = rv, lv

    def commit(move):
        for j, dunits in move:
            old_u = units[j]
            for i in columns[j]:
                radial[i] += rad_m[i][j] * dunits / UNIT
                lateral[i] += lat_m[i][j] * dunits / UNIT
            tension[j] += k_m[j] * dunits / UNIT
            units[j] = old_u + dunits

    def official_key():
        sm = summarize(spokes, weights, radial, lateral, tension, limits,
                       include_violations=False)
        return metrics_key(sm, sum(abs(u) for u in units) / UNIT)

    # ---- 阶段一：平滑引导 ----
    soft_best = _soft_key(spokes, weights, radial, lateral, tension, limits,
                          units, rad_tol, lat_tol)
    for _ in range(cap):
        best = None
        for move in all_moves():
            snap = probe(move)
            sc = _soft_key(spokes, weights, radial, lateral, tension, limits,
                           units, rad_tol, lat_tol)
            restore(snap)
            if sc < soft_best - 1e-12 and (best is None or sc < best[0]):
                best = (sc, move)
        if best is None:
            break
        soft_best = best[0]
        commit(best[1])

    # ---- 阶段二：官方字典序精修（允许持平行走跨越单步平台）----
    anchor_key = official_key()
    best_head3 = anchor_key[:3]
    best_snap = (units[:], list(radial), list(lateral), list(tension))
    plateau = 0
    plateau_budget = max(16, n)
    for _ in range(cap):
        strict, flat = None, None
        for move in all_moves():
            snap = probe(move)
            key = official_key()
            restore(snap)
            # 严格改进：前三项（超限量/峰值/离散度）变小，无论总转动量
            if key[:3] < anchor_key[:3] and (strict is None or key < strict[0]):
                strict = (key, move)
            # 平台动作：前三项不恶化（允许数值微小变化），在可行动作中
            # 取前三项最小者，用于走到需多孔协调的配置
            if key[:3] <= anchor_key[:3] and (flat is None or key < flat[0]):
                flat = (key, move)
        if strict is None and plateau < plateau_budget and flat is not None:
            key, move = flat
            plateau += 1
            commit(move)
            if key[:3] < best_head3:
                best_head3 = key[:3]
                best_snap = (units[:], list(radial), list(lateral), list(tension))
            # 平台动作前三项若严格变小，推进锚点继续搜索
            if key[:3] < anchor_key[:3]:
                anchor_key = key
                plateau = 0
            continue
        if strict is None:
            break
        key, move = strict
        commit(move)
        anchor_key = key
        best_head3 = key[:3]
        best_snap = (units[:], list(radial), list(lateral), list(tension))
        plateau = 0

    # 平台行走可能走过头：恢复沿途前三项最佳的状态
    best_units, radial_b, lateral_b, tension_b = best_snap
    radial[:], lateral[:], tension[:] = radial_b, lateral_b, tension_b
    return best_units, None


def _build_steps(spokes, inf, radial0, lateral0, tension0, weights, limits,
                 units: list[int], start_angle: float) -> list[dict]:
    """把整轮动作编排为可执行步进。

    每孔转动量分解为分层 chunk 序列（先 1/2 圈、再 1/4、最后 1/8，
    每孔每层至多一个 chunk）。按层展开，层内自起始角沿角序、左右就近
    配对交错，避免先把单孔深拧到位。逐步预测执行后的跳动与左右张力。
    """
    n = len(spokes)

    def angle_key(i):
        return (spokes[i]["angle_rad"] - start_angle) % (2.0 * math.pi)

    active = [i for i in range(n) if units[i] != 0]
    by_side = {
        "right": sorted((i for i in active if spokes[i]["side"] == "right"),
                        key=angle_key),
        "left": sorted((i for i in active if spokes[i]["side"] == "left"),
                       key=angle_key),
    }

    def signed(i, chunk):
        return (1 if units[i] > 0 else -1) * chunk

    actions: list[tuple[int, int]] = []
    consumed = {i: 0 for i in active}  # 已被更大档消耗的单位量
    for chunk in (4, 2, 1):
        # 该层每孔贡献一个 chunk（按扣除更大档后的剩余量），直到该档取尽
        depth = {i: (abs(units[i]) - consumed[i]) // chunk for i in active}
        while any(v > 0 for v in depth.values()):
            layer_r = [i for i in by_side["right"] if depth[i] > 0]
            layer_l = [i for i in by_side["left"] if depth[i] > 0]
            used_l = set()
            layer: list[tuple[int, int]] = []
            for i in layer_r:
                layer.append((i, signed(i, chunk)))
                depth[i] -= 1
                consumed[i] += chunk
                if layer_l:
                    k = min(
                        (q for q in layer_l if q not in used_l),
                        key=lambda q: abs(math.atan2(
                            math.sin(spokes[q]["angle_rad"] - spokes[i]["angle_rad"]),
                            math.cos(spokes[q]["angle_rad"] - spokes[i]["angle_rad"]))),
                        default=None,
                    )
                    if k is not None:
                        layer.append((k, signed(k, chunk)))
                        used_l.add(k)
                        depth[k] -= 1
                        consumed[k] += chunk
            for k in layer_l:
                if k not in used_l:
                    layer.append((k, signed(k, chunk)))
                    depth[k] -= 1
                    consumed[k] += chunk
            actions.extend(layer)

    steps = []
    cur_units = [0] * n
    for k, (j, du) in enumerate(actions, start=1):
        cur_units[j] += du
        r, l, t = _apply_units(radial0, lateral0, tension0, inf, cur_units)
        sm = summarize(spokes, weights, r, l, t, limits, include_violations=False)
        m = dict(sm["metrics"])
        m["total_turns"] = r3(sum(abs(x) for x in cur_units) / UNIT)
        steps.append({
            "step": k,
            "rim_hole": spokes[j]["rim_hole"],
            "side": spokes[j]["side"],
            "direction": "tighten" if du > 0 else "loosen",
            "turns": r3(abs(du) / UNIT),
            "predicted": m,
        })
    return steps


def build_candidates(batch: dict, spokes: list[dict], weights: list[float],
                     inf: dict, analysis: dict, start_angle: float) -> list[dict]:
    """生成并排序候选方案；候选 0 恒为不动作基线（可直接确认冻结本轮测量）。"""
    n = len(spokes)
    limits = _limits_arrays(batch, spokes)
    pmap = {p["rim_hole"]: i for i, p in enumerate(spokes)}
    radial0 = [None] * n
    lateral0 = [None] * n
    tension0 = [None] * n
    for p in analysis["points"]:
        i = pmap[p["rim_hole"]]
        radial0[i] = p["radial_mm"]
        lateral0[i] = p["lateral_mm"]
        tension0[i] = p["tension_n"]

    locks = {h for h in batch.get("locks", {}) if isinstance(h, int)}
    max_units = int(round(batch["max_turns_per_spoke"] * UNIT))

    def package(label, units):
        r, l, t = _apply_units(radial0, lateral0, tension0, inf, units)
        sm = summarize(spokes, weights, r, l, t, limits)
        total_turns = sum(abs(x) for x in units) / UNIT
        sm["metrics"]["total_turns"] = r3(total_turns)
        steps = _build_steps(spokes, inf, radial0, lateral0, tension0,
                             weights, limits, units, start_angle)
        per_spoke = []
        blocked = []
        for j, s in enumerate(spokes):
            hole = s["rim_hole"]
            if units[j] != 0:
                per_spoke.append({
                    "rim_hole": hole, "side": s["side"],
                    "turns": r3(abs(units[j]) / UNIT),
                    "direction": "tighten" if units[j] > 0 else "loosen",
                    "tension_before_n": r3(tension0[j]),
                    "tension_after_n": r3(t[j]),
                    "tension_delta_n": r3(t[j] - tension0[j]),
                })
            reasons = []
            if hole in locks:
                reasons.append("locked")
            if tension0[j] >= limits["tension_max_n"][j] - 1e-9:
                reasons.append("at_upper_limit_no_tighten")
            if tension0[j] <= limits["tension_min_n"][j] + 1e-9:
                reasons.append("at_lower_limit_no_loosen")
            if reasons:
                blocked.append({"rim_hole": hole, "side": s["side"], "reasons": reasons})
        return {
            "label": label,
            "metrics": sm["metrics"],
            "tension_by_side": sm["tension_by_side"],
            "violations": sm["violations"],
            "spoke_actions": per_spoke,
            "blocked_spokes": blocked,
            "steps": steps,
        }

    variants = [
        ("quarter_eighth_half", {4, 2, 1}),
        ("eighth_quarter", {1, 2}),
    ]
    results = [package("no_action", [0] * n)]
    baseline = summarize(spokes, weights, radial0, lateral0, tension0, limits,
                         include_violations=False)
    baseline_key = metrics_key(baseline, 0.0)
    seen = {tuple([0] * n)}
    for label, step_set in variants:
        units, _ = _search_variant(spokes, weights, radial0, lateral0, tension0,
                                   limits, inf, locks, max_units, step_set)
        key = tuple(units)
        if key in seen or not any(units):
            continue
        r, l, t = _apply_units(radial0, lateral0, tension0, inf, units)
        sm = summarize(spokes, weights, r, l, t, limits, include_violations=False)
        total = sum(abs(x) for x in units) / UNIT
        # 代理/平台搜索只是手段：官方字典序不严格优于不动作基线的候选不输出
        if not metrics_key(sm, total) < baseline_key:
            continue
        seen.add(key)
        results.append(package(label, units))

    ranked = [results[0]] + sorted(
        results[1:],
        key=lambda c: (c["metrics"]["violation_count"],
                       c["metrics"]["max_runout_mm"],
                       c["metrics"]["local_tension_dispersion_n"],
                       c["metrics"]["total_turns"]),
    )
    for idx, cand in enumerate(ranked):
        cand["candidate_index"] = idx
    return ranked
