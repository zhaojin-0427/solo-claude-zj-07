"""轮组几何：辐条长度、入圈角、法兰出线角、张力比。

坐标系：车轴沿 z 轴，z 向右（驱动侧）为正；轮平面为 z=0。
圈孔 i 的角度 a_r = 2*pi*i/N（从右侧看顺时针为正，仅影响相对相位）。
右法兰孔 j 的角度 a_h = 2*pi*j/n；左法兰孔与右法兰错开半个节距：a_h = 2*pi*(j+0.5)/n。
"""

from __future__ import annotations

import math

from .errors import WheelError

FORMULA_VERSION = "wheel-geometry/1.1"

# 随版本快照一起保存的计算公式（文本形式，便于追溯）
FORMULAS = {
    "spoke_length": "L = sqrt(R^2 + r_eff^2 + w_eff^2 - 2*R*r_eff*cos(delta)) - s/2",
    "delta": "delta = |a_rim - a_hub|（实际孔位夹角；标准穿法下 = 2*pi*cross/n, n = 每侧孔数）",
    "r_eff": "r_eff = PCD/2 - d/2（内穿 heads-in）或 PCD/2 + d/2（外穿 heads-out），d = 辐条杆径",
    "w_eff": "w_eff = center_to_flange - rim_hole_offset（同侧圈孔横向偏移）",
    "bracing_angle": "sin(bracing) = w_eff / L_raw, L_raw = L + s/2",
    "tension_ratio": "T_left / T_right = sin(bracing_right) / sin(bracing_left)（横向力平衡）",
    "rim_entry_angle": "入圈角 = 辐条方向与圈孔处半径方向的夹角（0 = 正对轴心）",
    "flange_exit_angle": "出线角 = 辐条在轮平面投影与法兰孔处切线的夹角（0 = 相切，90 = 径向）",
    "washer_fit": "err_hole = spoke_length - washer_t - ideal_hole（逐孔判定）；max|err_hole| <= length_tolerance",
    "thread_engagement": "engagement = spoke_thread_length + min(0, min err_hole) >= min_thread_engagement; max err_hole <= max_protrusion",
    "inventory_merge": "同一长度的多行库存数量累加后，与两侧各 n 根的需求量比较",
}


def r3(x: float) -> float:
    return round(float(x), 3)


def rim_side(i: int) -> str:
    return "right" if i % 2 == 0 else "left"


def rim_angle(i: int, n_total: int) -> float:
    return 2.0 * math.pi * i / n_total


def hub_angle(side: str, j: int, n_side: int) -> float:
    if side == "right":
        return 2.0 * math.pi * j / n_side
    return 2.0 * math.pi * (j + 0.5) / n_side


def spoke_length(rim_r: float, flange_r: float, w_eff: float, delta: float, hole_dia: float) -> float:
    """标准辐条长度公式（含法兰孔径修正 s/2）。"""
    return math.sqrt(
        rim_r * rim_r + flange_r * flange_r + w_eff * w_eff
        - 2.0 * rim_r * flange_r * math.cos(delta)
    ) - hole_dia / 2.0


def validate_counts(spec) -> int:
    """孔数校验，返回每侧孔数 n。"""
    n_total = spec.rim.holes
    if n_total % 4 != 0:
        raise WheelError(
            "HOLE_COUNT_INVALID",
            f"轮圈孔数 {n_total} 必须能被 4 整除（每侧需偶数孔才能均分顺/逆向辐条）",
            {"rim_holes": n_total},
        )
    per_flange = spec.hub.holes_per_flange
    if 2 * per_flange != n_total:
        raise WheelError(
            "HOLE_COUNT_MISMATCH",
            f"轮圈 {n_total} 孔与花鼓两侧共 {2 * per_flange} 孔不匹配，无法逐孔对应",
            {
                "rim_holes": n_total,
                "hub_holes_per_flange": per_flange,
                "hub_holes_total": 2 * per_flange,
            },
        )
    if spec.rim.valve_position >= n_total:
        raise WheelError(
            "VALVE_POSITION_INVALID",
            f"阀孔位置 {spec.rim.valve_position} 超出圈孔范围 0..{n_total - 1}",
            {"valve_position": spec.rim.valve_position, "rim_holes": n_total},
        )
    return n_total // 2


def max_cross(n_side: int) -> int:
    """每侧 n 孔时的实用最大交叉数（再大则辐条近乎相切、条帽角度无法容纳）。"""
    return (n_side - 2) // 4


def check_cross(spec, n_side: int) -> None:
    k_max = max_cross(n_side)
    for side in ("left", "right"):
        lac = getattr(spec, side)
        if lac.cross > k_max:
            raise WheelError(
                "CROSS_INFEASIBLE",
                f"{side} 侧 {lac.cross}x 交叉不可行：每侧 {n_side} 孔最多支持 {k_max}x"
                f"（再大则入圈角过小、相邻辐条在法兰处干涉）",
                {"side": side, "cross": lac.cross, "holes_per_side": n_side, "max_cross": k_max},
            )


def _side_params(spec, side: str) -> dict:
    rim, hub = spec.rim, spec.hub
    if side == "right":
        w, o = hub.center_to_flange_right_mm, rim.hole_offset_right_mm
        r = hub.flange_pcd_right_mm / 2.0
        lac = spec.right
    else:
        w, o = hub.center_to_flange_left_mm, rim.hole_offset_left_mm
        r = hub.flange_pcd_left_mm / 2.0
        lac = spec.left
    w_eff = w - o
    if w_eff <= 0:
        raise WheelError(
            "GEOMETRY_INVALID",
            f"{side} 侧有效法兰偏距 w_eff = {r3(w_eff)} mm <= 0（中心偏距须大于圈孔偏移）",
            {"side": side, "center_to_flange_mm": w, "rim_hole_offset_mm": o},
        )
    return {
        "rim_r": rim.erd_mm / 2.0,
        "flange_r": r,
        "w": w,
        "offset": o,
        "w_eff": w_eff,
        "cross": lac.cross,
        "heads_in": lac.heads_in,
    }


def _clamp(x: float) -> float:
    return max(-1.0, min(1.0, x))


def compute_geometry(spec, entries: list[dict], n_side: int) -> dict:
    """计算侧级汇总与逐孔结果。entries 为 lacing.build_mapping 生成的映射。"""
    n_total = spec.rim.holes
    s = spec.hub.spoke_hole_diameter_mm
    d = spec.spoke_diameter_mm
    rim_r = spec.rim.erd_mm / 2.0

    sides: dict[str, dict] = {}
    for side in ("left", "right"):
        p = _side_params(spec, side)
        r = p["flange_r"]
        if rim_r <= r:
            raise WheelError(
                "GEOMETRY_INVALID",
                f"{side} 侧法兰半径 {r3(r)} mm 不小于轮圈半径 {r3(rim_r)} mm",
                {"side": side, "flange_r_mm": r3(r), "rim_r_mm": r3(rim_r)},
            )
        alpha = 2.0 * math.pi * p["cross"] / n_side
        l_base = spoke_length(rim_r, r, p["w_eff"], alpha, s)
        l_in = spoke_length(rim_r, r - d / 2.0, p["w_eff"], alpha, s)
        l_out = spoke_length(rim_r, r + d / 2.0, p["w_eff"], alpha, s)
        bracing = math.degrees(math.asin(_clamp(p["w_eff"] / (l_base + s / 2.0))))
        sides[side] = {
            "cross": p["cross"],
            "heads_in": p["heads_in"],
            "alpha_deg": r3(math.degrees(alpha)),
            "center_to_flange_mm": r3(p["w"]),
            "rim_hole_offset_mm": r3(p["offset"]),
            "w_eff_mm": r3(p["w_eff"]),
            "flange_pcd_mm": r3(2.0 * r),
            "spoke_length_mm": r3(l_base),
            "spoke_length_heads_in_mm": r3(l_in),
            "spoke_length_heads_out_mm": r3(l_out),
            "bracing_angle_deg": r3(bracing),
            "_sin_bracing": math.sin(math.radians(bracing)),
        }

    sin_l, sin_r = sides["left"]["_sin_bracing"], sides["right"]["_sin_bracing"]
    ratio = sin_r / sin_l if sin_l > 0 else None

    per_hole = []
    for e in entries:
        side = e["side"]
        p = _side_params(spec, side)
        r = p["flange_r"]
        a_r = rim_angle(e["rim_hole"], n_total)
        a_h = hub_angle(side, e["hub_hole"], n_side)
        z_rim = p["offset"] if side == "right" else -p["offset"]
        z_hub = p["w"] if side == "right" else -p["w"]
        rim_pos = (rim_r * math.cos(a_r), rim_r * math.sin(a_r), z_rim)
        hub_pos = (r * math.cos(a_h), r * math.sin(a_h), z_hub)

        delta = abs(math.atan2(math.sin(a_r - a_h), math.cos(a_r - a_h)))
        r_eff = r - d / 2.0 if e["insertion"] == "heads_in" else r + d / 2.0
        length = spoke_length(rim_r, r_eff, p["w_eff"], delta, s)

        # 入圈角：辐条（圈->毂）与圈孔处向心方向的夹角
        v = (hub_pos[0] - rim_pos[0], hub_pos[1] - rim_pos[1], hub_pos[2] - rim_pos[2])
        v_norm = math.sqrt(v[0] ** 2 + v[1] ** 2 + v[2] ** 2)
        u_in = (-math.cos(a_r), -math.sin(a_r), 0.0)
        cos_entry = _clamp((v[0] * u_in[0] + v[1] * u_in[1]) / v_norm)
        entry_deg = math.degrees(math.acos(cos_entry))

        # 法兰出线角：辐条轮平面投影与法兰孔切线的夹角
        px, py = rim_pos[0] - hub_pos[0], rim_pos[1] - hub_pos[1]
        p_norm = math.hypot(px, py)
        tx, ty = -math.sin(a_h), math.cos(a_h)
        exit_deg = math.degrees(math.acos(_clamp(abs(px * tx + py * ty) / p_norm))) if p_norm > 0 else 90.0

        bracing_deg = math.degrees(math.asin(_clamp(p["w_eff"] / v_norm)))

        per_hole.append({
            "rim_hole": e["rim_hole"],
            "side": side,
            "hub_hole": e["hub_hole"],
            "direction": e["direction"],
            "insertion": e["insertion"],
            "length_mm": r3(length),
            "rim_entry_angle_deg": r3(entry_deg),
            "flange_exit_angle_deg": r3(exit_deg),
            "bracing_angle_deg": r3(bracing_deg),
        })

    per_hole.sort(key=lambda x: x["rim_hole"])
    for side in ("left", "right"):
        sides[side].pop("_sin_bracing")

    return {
        "holes_total": n_total,
        "holes_per_side": n_side,
        "rim_radius_mm": r3(rim_r),
        "spoke_hole_diameter_mm": r3(s),
        "spoke_diameter_mm": r3(d),
        "sides": sides,
        "tension_ratio_left_to_right": r3(ratio) if ratio is not None else None,
        "per_hole": per_hole,
    }
