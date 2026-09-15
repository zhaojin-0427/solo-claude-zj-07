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
    "violation_count": "超限测点数按**唯一孔位**计数：同一孔同时径向/横向/张力超限只计 1，各项列入该孔 issues",
    "step_granularity": "条帽步进只允许 1/8 圈（45°，1 单位）或 1/4 圈（90°，2 单位）；编排按层铺放（先 1/4 再 1/8），层内左右就近配对交错，逐步预测跳动与左右张力",
    "immovable_spokes": "锁定孔或初始张力已达上限的孔，收紧/拧松都不调整；初始达下限的孔允许收紧、只禁止拧松；均列入 blocked_spokes",
    "search": "阶段一平滑目标（超限平方+残差平方+碟形/偏心全局项）引导越门限与多孔协调；阶段二按官方字典序精修（含持平行走），阶段一原始结果与精修结果都参与候选",
    "candidate_ranking": "**全部候选（含不动作基线）统一**按字典序排列，不固定置顶：超限测点数↑ → 最大跳动 hypot(径,横)↑ → 局部张力离散度↑ → 总转动量↑（全部取小）",
    "confirm": "POST rounds/confirm 省略 candidate_index 时始终确认 no_action（不论其排序位置）；显式索引按候选列表位置确认",
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

    # 超限测点按**唯一孔位**统计：同一孔同时有径向/横向/张力超限，
    # 只计 1 个超限测点；具体各项保留在该孔的 issues 中。
    violations = []
    for i, s in enumerate(name_idx):
        issues = []
        if abs(res_r[i]) > limits["radial_tolerance_mm"] + 1e-9:
            issues.append({"type": "radial_runout", "value_mm": r3(res_r[i]),
                           "limit_mm": r3(limits["radial_tolerance_mm"])})
        if abs(res_l[i]) > limits["lateral_tolerance_mm"] + 1e-9:
            issues.append({"type": "lateral_runout", "value_mm": r3(res_l[i]),
                           "limit_mm": r3(limits["lateral_tolerance_mm"])})
        if tension[i] > limits["tension_max_n"][i] + 1e-9:
            issues.append({"type": "tension_over_max", "value_n": r3(tension[i]),
                           "limit_n": r3(limits["tension_max_n"][i])})
        if tension[i] < limits["tension_min_n"][i] - 1e-9:
            issues.append({"type": "tension_under_min", "value_n": r3(tension[i]),
                           "limit_n": r3(limits["tension_min_n"][i])})
        if issues:
            violations.append({
                "rim_hole": s["rim_hole"], "side": s["side"],
                "issue_count": len(issues), "issues": issues,
            })

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


def _fast_head3(spokes, weights, radial, lateral, tension, limits,
                side_order, win_index):
    """精修热路径用：官方键前三项（超限量/最大跳动/离散度）。"""
    violations, peak_sq, mean_r = _fast_head2(
        spokes, weights, radial, lateral, tension, limits)
    disp = _fast_dispersion(tension, side_order, win_index)
    return violations, r3(math.sqrt(peak_sq)), disp


def _fast_head2(spokes, weights, radial, lateral, tension, limits):
    """廉价前两项：超限测点数（唯一孔位）与最大跳动平方、径向基线。"""
    n = len(spokes)
    acc = 0.0
    for i in range(n):
        acc += weights[i] * radial[i]
    mean_r = acc
    rad_tol = limits["radial_tolerance_mm"]
    lat_tol = limits["lateral_tolerance_mm"]
    tmax = limits["tension_max_n"]
    tmin = limits["tension_min_n"]
    violations = 0
    peak_sq = 0.0
    for i in range(n):
        rr = radial[i] - mean_r
        ll = lateral[i]
        if (abs(rr) > rad_tol + 1e-9 or abs(ll) > lat_tol + 1e-9
                or tension[i] > tmax[i] + 1e-9 or tension[i] < tmin[i] - 1e-9):
            violations += 1
        comb = rr * rr + ll * ll
        if comb > peak_sq:
            peak_sq = comb
    return violations, peak_sq, mean_r


def _fast_dispersion(tension, side_order, win_index):
    """同侧 5 孔滑窗张力标准差最大值。"""
    disp_max = 0.0
    for side in ("left", "right"):
        idxs = side_order[side]
        m = len(idxs)
        if m <= LOCAL_WINDOW:
            continue
        for wi in range(m):
            s = 0.0
            for d in win_index:
                s += tension[idxs[(wi + d) % m]]
            mean = s / LOCAL_WINDOW
            var = 0.0
            for d in win_index:
                v = tension[idxs[(wi + d) % m]] - mean
                var += v * v
            std = math.sqrt(var / LOCAL_WINDOW)
            if std > disp_max:
                disp_max = std
    return r3(disp_max)


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
              rad_tol, lat_tol, cos_a=None, sin_a=None):
    """平滑引导目标（仅搜索内部使用）。

    超限平方和 + 全残差平方和（小权重）+ 碟形/偏心全局分量 +
    张力越界重罚 + 微小转动量正则。让单孔步进在尚未把测点压过门限、
    或必须多孔协调（碟形/偏心）时仍有明确下山方向。
    cos_a/sin_a 可由调用方预计算传入以加速热路径。
    最终方案优劣一律以官方字典序（见 metrics_key）为准。
    """
    n = len(spokes)
    if cos_a is None:
        cos_a = [math.cos(s["angle_rad"]) for s in spokes]
        sin_a = [math.sin(s["angle_rad"]) for s in spokes]
    mean_r = 0.0
    mean_l = 0.0
    for i in range(n):
        mean_r += weights[i] * radial[i]
        mean_l += weights[i] * lateral[i]
    e_x = 0.0
    e_y = 0.0
    excess = 0.0
    residual2 = 0.0
    t_pen = 0.0
    tmax = limits["tension_max_n"]
    tmin = limits["tension_min_n"]
    reg = 0.0
    for i in range(n):
        dr = radial[i] - mean_r
        ll = lateral[i]
        e_x += weights[i] * dr * cos_a[i]
        e_y += weights[i] * dr * sin_a[i]
        er = abs(dr) - rad_tol
        el = abs(ll) - lat_tol
        if er > 0:
            excess += er * er
        if el > 0:
            excess += el * el
        residual2 += dr * dr + ll * ll
        ti = tension[i]
        if ti > tmax[i]:
            d = ti - tmax[i]
            t_pen += (d * 0.01) ** 2
        elif ti < tmin[i]:
            d = tmin[i] - ti
            t_pen += (d * 0.01) ** 2
        u = units[i] / UNIT
        reg += u * u
    global_term = 2.0 * mean_l * mean_l + 2.0 * (e_x * e_x + e_y * e_y)
    return excess + 0.02 * residual2 + global_term + 100.0 * t_pen + 1e-6 * reg


def _search_variant(spokes, weights, radial0, lateral0, tension0, limits, inf,
                    locks, max_units, step_units_set, cap=400):
    """两阶段贪心，返回若干个整轮转动方案（1/8 圈整数单位），由调用方统一排序。

    锁定孔与初始张力已达**上限**的孔完全不参与任何方向的调整；初始达
    **下限**的孔允许收紧，feasible() 只禁止其拧松。阶段一（引导）：单孔 +
        左右成对动作沿平滑目标下山，完成碟形/偏心所需的多孔协调（可能以
        张力离散度换跳动）；阶段二（精修）：在阶段一终点按官方四项键
        严格变优地继续清除残留局部跳动（含少量持平行走）；阶段一原始
        结果与精修结果都返回，全部方案最终由统一字典序裁决。
    """
    n = len(spokes)
    units = [0] * n
    radial, lateral, tension = list(radial0), list(lateral0), list(tension0)
    rad_tol = limits["radial_tolerance_mm"]
    lat_tol = limits["lateral_tolerance_mm"]
    columns = inf["columns"]
    rad_m, lat_m, k_m = inf["radial"], inf["lateral"], inf["stiffness_n_per_turn"]
    cos_a = [math.cos(s["angle_rad"]) for s in spokes]
    sin_a = [math.sin(s["angle_rad"]) for s in spokes]

    def soft_key():
        return _soft_key(spokes, weights, radial, lateral, tension, limits,
                         units, rad_tol, lat_tol, cos_a, sin_a)

    # 不可调整孔（两个方向都禁止）：锁定孔，或本轮**初始**张力已达上限的孔。
    # 初始达下限的孔不放入此集合：允许收紧，只在 feasible() 中拒绝拧松
    # （下限孔净动作只会为正，张力不会跌破初始值）。
    immovable = set(locks)
    for j in range(n):
        if tension0[j] >= limits["tension_max_n"][j] - 1e-9:
            immovable.add(spokes[j]["rim_hole"])

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
    pairs = sorted(
        (j, k) for j, k in local_pairs
        if spokes[j]["rim_hole"] not in immovable
        and spokes[k]["rim_hole"] not in immovable
    )

    def feasible(j, sign, du_units):
        # 动作过程中也不得越过张力窗口（达限孔已在 immovable 中整体排除）
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
            if spokes[j]["rim_hole"] in immovable:
                continue
            for step in step_units_set:
                for sign in (1, -1):
                    if feasible(j, sign, step):
                        moves.append(((j, sign * step),))
        for j, k in pairs:
            for step in step_units_set:
                for sign in (1, -1):
                    if feasible(j, sign, step) and feasible(k, -sign, step):
                        moves.append(((j, sign * step), (k, -sign * step)))
        return moves

    def probe(move):
        """施加动作，返回动作前快照；由 restore 一次性还原。

        同一动作内多根辐条的钟形窗可能交叠，因此受影响观测孔的动作前
        值只记录一次（不能按孔分别快照后顺序恢复，否则交叠孔会被覆盖成
        已被前一孔修改的值，污染后续试算）。
        """
        touched_idx = set()
        acted = {}
        for j, dunits in move:
            if j in acted:
                continue
            acted[j] = dunits
            touched_idx.update(columns[j])
        old_radial = {i: radial[i] for i in touched_idx}
        old_lateral = {i: lateral[i] for i in touched_idx}
        old_tension = {j: tension[j] for j in acted}
        old_units = {j: units[j] for j in acted}
        for j, dunits in acted.items():
            units[j] += dunits
            for i in columns[j]:
                radial[i] += rad_m[i][j] * dunits / UNIT
                lateral[i] += lat_m[i][j] * dunits / UNIT
            tension[j] += k_m[j] * dunits / UNIT
        return acted, old_units, old_tension, old_radial, old_lateral

    def restore(snap):
        acted, old_units, old_tension, old_radial, old_lateral = snap
        for j, old_u in old_units.items():
            units[j] = old_u
        for j, old_t in old_tension.items():
            tension[j] = old_t
        for i, rv in old_radial.items():
            radial[i] = rv
        for i, lv in old_lateral.items():
            lateral[i] = lv

    def commit(move):
        seen = {}
        for j, dunits in move:
            seen[j] = seen.get(j, 0) + dunits
        for j, dunits in seen.items():
            old_u = units[j]
            for i in columns[j]:
                radial[i] += rad_m[i][j] * dunits / UNIT
                lateral[i] += lat_m[i][j] * dunits / UNIT
            tension[j] += k_m[j] * dunits / UNIT
            units[j] = old_u + dunits

    # 预计算同侧角序索引与 5 孔滑窗偏移，供精修热路径复用
    side_order = {
        "left": [i for i in range(n) if spokes[i]["side"] == "left"],
        "right": [i for i in range(n) if spokes[i]["side"] == "right"],
    }
    win_index = list(range(-(LOCAL_WINDOW // 2), LOCAL_WINDOW // 2 + 1))

    def official_key():
        sm = summarize(spokes, weights, radial, lateral, tension, limits,
                       include_violations=False)
        return metrics_key(sm, sum(abs(u) for u in units) / UNIT)

    def total_turns():
        return sum(abs(u) for u in units) / UNIT

    # ---- 阶段一：平滑引导（多孔协调纠正碟形/偏心，可能以离散度换跳动）----
    soft_best = soft_key()
    for _ in range(cap):
        best = None
        for move in all_moves():
            snap = probe(move)
            sc = soft_key()
            restore(snap)
            if sc < soft_best - 1e-12 and (best is None or sc < best[0]):
                best = (sc, move)
        if best is None:
            break
        soft_best = best[0]
        commit(best[1])
    phase1_units = units[:]

    def snapshot_state():
        return units[:], list(radial), list(lateral), list(tension)

    def restore_state(snap):
        u, r, l, t = snap
        units[:] = u
        radial[:], lateral[:], tension[:] = r, l, t

    def head3():
        viol, peak_sq, _ = _fast_head2(spokes, weights, radial, lateral,
                                       tension, limits)
        disp = _fast_dispersion(tension, side_order, win_index)
        return viol, r3(math.sqrt(peak_sq)), disp

    def head2():
        viol, peak_sq, _ = _fast_head2(spokes, weights, radial, lateral,
                                       tension, limits)
        return viol, r3(math.sqrt(peak_sq))

    def official_refine(start_snap, refine_cap=80):
        """阶段二：从 start 起按官方字典序精修（含持平行走）。

        热路径先用廉价的 head2（超限量/峰值）筛动作，只有前两项不劣于
        锚点时才计算昂贵的离散度形成完整三项键；只接受完整四项键严格
        变优的状态，因此既不回退起点的多孔协调，也能清除残留局部跳动。
        """
        restore_state(start_snap)
        best_h3 = head3()
        best_turns = total_turns()
        best_snap = snapshot_state()
        anchor_h2 = best_h3[:2]
        anchor_disp = best_h3[2]
        plateau = 0
        stale = 0
        for _ in range(refine_cap):
            strict, flat = None, None
            for move in all_moves():
                snap = probe(move)
                h2 = head2()
                # 前两项已劣于锚点：不可能入选，跳过昂贵的离散度计算
                if h2 > anchor_h2:
                    restore(snap)
                    continue
                disp = _fast_dispersion(tension, side_order, win_index)
                h3 = (h2[0], h2[1], disp)
                tt = total_turns()
                restore(snap)
                if h2 <= anchor_h2 and (flat is None or (h3, tt) < flat[0]):
                    flat = ((h3, tt), move)
                if h2 < anchor_h2 and (strict is None or (h3, tt) < strict[0]):
                    strict = ((h3, tt), move)
            if strict is not None:
                (h3, tt), move = strict
                commit(move)
                anchor_h2 = h3[:2]
                anchor_disp = h3[2]
                plateau = 0
            elif plateau < plateau_budget and flat is not None:
                (h3, tt), move = flat
                commit(move)
                plateau += 1
                if h3[:2] < anchor_h2:
                    anchor_h2 = h3[:2]
                    anchor_disp = h3[2]
                    plateau = 0
            else:
                break
            cur_h3, cur_tt = head3(), total_turns()
            if (cur_h3, cur_tt) < (best_h3, best_turns):
                best_h3, best_turns = cur_h3, cur_tt
                best_snap = snapshot_state()
                stale = 0
                if best_h3[0] == 0 and best_h3[1] <= max(rad_tol, lat_tol) + 1e-9:
                    break
            else:
                stale += 1
                if stale >= max(4, n // 4):
                    break
        return (best_h3, best_turns), best_snap

    plateau_budget = max(6, n // 4)
    # 阶段一终点必须先深拷贝保存：official_refine 会在同一组可变数组上恢复状态
    phase1_snap = (phase1_units[:],
                   [radial0[i] + sum(rad_m[i][j] * phase1_units[j] / UNIT
                                     for j in range(n)) for i in range(n)],
                   [lateral0[i] + sum(lat_m[i][j] * phase1_units[j] / UNIT
                                      for j in range(n)) for i in range(n)],
                   [tension0[j] + k_m[j] * phase1_units[j] / UNIT for j in range(n)])
    solutions = [phase1_units[:]]
    seen = {tuple(phase1_units)}
    # 在阶段一终点上做一次官方字典序精修（软目标已覆盖纯局部跳动场景，
    # 无需从零再跑一条重复轨迹）
    _, refined_snap = official_refine(phase1_snap)
    refined_units = refined_snap[0]
    if tuple(refined_units) not in seen and any(refined_units):
        seen.add(tuple(refined_units))
        solutions.append(list(refined_units))
    return solutions


def _build_steps(spokes, inf, radial0, lateral0, tension0, weights, limits,
                 units: list[int], start_angle: float) -> list[dict]:
    """把整轮动作编排为可执行步进。

    条帽步进只允许 **1/4 圈（2 单位）与 1/8 圈（1 单位）**。
    每孔转动量按层铺放（先 1/4、再 1/8，每孔每层至多一步），层内自
    起始角沿角序、左右就近配对交错，避免先把单孔深拧到位。逐步预测
    执行后的跳动、**左右侧张力**与累计转动量。
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
    for chunk in (2, 1):
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
            "predicted_tension_by_side": sm["tension_by_side"],
        })
    return steps


def build_candidates(batch: dict, spokes: list[dict], weights: list[float],
                     inf: dict, analysis: dict, start_angle: float) -> list[dict]:
    """生成候选方案并**统一按四项字典序排序**（含不动作基线，不强制置顶）。

    排序键：超限测点数（按唯一孔位）→ 最大跳动 → 局部张力离散度 → 总转动量。
    动作步进只允许 1/8 与 1/4 圈。锁定孔与初始张力达上/下限的孔不参与调整，
    各候选在 blocked_spokes 中给出原因（locked / at_tension_limit）。
    """
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

    # 初始达上限：两个方向都不调整；初始达下限：只允许收紧、禁止拧松
    at_upper = {
        s["rim_hole"] for j, s in enumerate(spokes)
        if tension0[j] >= limits["tension_max_n"][j] - 1e-9
    }
    at_lower = {
        s["rim_hole"] for j, s in enumerate(spokes)
        if tension0[j] <= limits["tension_min_n"][j] + 1e-9
    }

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
            if hole in at_upper:
                reasons.append("at_upper_tension_limit")
            if hole in at_lower:
                reasons.append("at_lower_tension_limit_no_loosen")
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
            "_rank_key": (
                sm["metrics"]["violation_count"],
                sm["metrics"]["max_runout_mm"],
                sm["metrics"]["local_tension_dispersion_n"],
                r3(total_turns),
            ),
        }

    # 两个搜索变体都只使用 1/8、1/4 圈步进；每个变体可能返回多个方案
    # （纯局部修正、保留碟形/偏心协调的精修、阶段一原始结果）
    variants = [
        ("eighth_and_quarter", {1, 2}),
        ("eighth_only", {1}),
    ]
    results = [package("no_action", [0] * n)]
    seen = {tuple([0] * n)}
    for label, step_set in variants:
        solutions = _search_variant(spokes, weights, radial0, lateral0, tension0,
                                   limits, inf, locks, max_units, step_set)
        for sol_no, units in enumerate(solutions):
            key = tuple(units)
            if key in seen or not any(units):
                continue
            seen.add(key)
            # 同一变体的多个方案以 _guided（阶段一协调结果）/
            # _refined（官方键精修后）区分
            tag = "" if len(solutions) == 1 else (
                "_guided" if sol_no == 0 else f"_{sol_no}")
            results.append(package(f"{label}{tag}", units))

    # 全部候选（含不动作基线）统一按四项字典序排列
    results.sort(key=lambda c: c["_rank_key"])
    for idx, cand in enumerate(results):
        cand["candidate_index"] = idx
        cand.pop("_rank_key")
    return results
