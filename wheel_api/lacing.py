"""孔位映射、阀孔避让、首根定位与编轮次序。

自动穿法按**实际孔位**搜索：枚举方向相位（phase 0/1，即顺/逆向组从哪个
角序位置开始）与每侧法兰对齐（shift 0..n-1，即孔位双射的整体旋转），
保留满足以下条件的候选：
- 交叉方向：每根辐条实际夹角符号与其顺/逆向一致；
- 交叉数：实际夹角折算的交叉数 k_eff = round(|delta| / 节距) 等于设置值；
- 阀孔净空：阀孔到最近辐条弦的距离 >= valve_clearance_min_mm。
在可行候选中取阀孔净空最大者（并列时取夹角散布最小、shift 最小者）；
无解时抛出 LACING_INFEASIBLE 并给出冲突孔。
"""

from __future__ import annotations

import math

from .errors import WheelError
from .geometry import (
    TWO_PI,
    check_cross,
    max_cross,
    next_rim_hole,
    r3,
    resolve_layout,
    validate_counts,
    valve_angle,
)


def _pt_seg_dist(p, a, b) -> float:
    ax, ay = a
    bx, by = b
    px, py = p
    dx, dy = bx - ax, by - ay
    if dx == 0 and dy == 0:
        return math.hypot(px - ax, py - ay)
    t = ((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy)
    t = max(0.0, min(1.0, t))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def _insertion(direction: str, heads_in: str) -> str:
    if direction == "radial":
        return "heads_in"
    return "heads_in" if direction == heads_in else "heads_out"


def _wrap_pi(x: float) -> float:
    return math.atan2(math.sin(x), math.cos(x))


def _spoke_chord(spec, layout, e: dict, rim_r: float):
    """辐条弦的轮平面端点：圈孔端 + 法兰孔端（均按实际角度）。"""
    r = (spec.hub.flange_pcd_right_mm if e["side"] == "right" else spec.hub.flange_pcd_left_mm) / 2.0
    a_r = layout.rim_angles[e["rim_hole"]]
    a_h = layout.hub_angles[e["side"]][e["hub_hole"]]
    return (
        (rim_r * math.cos(a_r), rim_r * math.sin(a_r)),
        (r * math.cos(a_h), r * math.sin(a_h)),
    )


def _valve_clearance_mm(spec, layout, entries: list[dict], v_pt=None) -> float:
    rim_r = spec.rim.erd_mm / 2.0
    if v_pt is None:
        v_ang = valve_angle(layout, spec.rim.valve_position)
        v_pt = (rim_r * math.cos(v_ang), rim_r * math.sin(v_ang))
    best = math.inf
    for e in entries:
        a, b = _spoke_chord(spec, layout, e, rim_r)
        best = min(best, _pt_seg_dist(v_pt, a, b))
    return best


def _side_phase_candidates(spec, layout, side: str, n_side: int, phase: int,
                           v_pt, rim_r: float) -> list[dict]:
    """枚举该侧在给定方向相位下的全部法兰对齐（shift 0..n-1）候选。

    映射按角序生成：角序第 m 个圈孔 -> 角序第 (m + shift ± cross) 个法兰孔，
    即在实际孔位上搜索孔位双射；每个候选附带交叉方向/交叉数冲突列表。
    """
    lac = getattr(spec, side)
    k = lac.cross
    pitch = TWO_PI / n_side
    order = [i for i in layout.rim_order if layout.rim_sides[i] == side]
    h_order = sorted(range(n_side), key=lambda j: (layout.hub_angles[side][j], j))
    cands = []
    for shift in range(n_side):
        entries, conflicts = [], []
        dev = 0.0
        for m, rim_id in enumerate(order):
            if k > 0:
                direction = "trailing" if (m + phase) % 2 == 0 else "leading"
            else:
                direction = "radial"
            hj = (m + shift - k) % n_side if direction == "leading" else (m + shift + k) % n_side
            hub_hole = h_order[hj]
            a_r = layout.rim_angles[rim_id]
            a_h = layout.hub_angles[side][hub_hole]
            delta = _wrap_pi(a_h - a_r)
            abs_delta = abs(delta)
            k_eff = int(round(abs_delta / pitch))
            if k > 0:
                want = 1.0 if direction == "trailing" else -1.0
                if delta * want <= 1e-9:
                    conflicts.append({
                        "rim_hole": rim_id,
                        "side": side,
                        "hub_hole": hub_hole,
                        "reason": "cross_direction",
                        "delta_deg": r3(math.degrees(delta)),
                        "expected_delta_deg": r3(math.degrees(want * k * pitch)),
                    })
            if k_eff != k:
                conflicts.append({
                    "rim_hole": rim_id,
                    "side": side,
                    "hub_hole": hub_hole,
                    "reason": "cross_count",
                    "cross_requested": k,
                    "cross_effective": k_eff,
                    "delta_deg": r3(math.degrees(delta)),
                })
            dev += abs(abs_delta - k * pitch)
            entries.append({
                "rim_hole": rim_id,
                "side": side,
                "hub_hole": hub_hole,
                "direction": direction,
                "insertion": _insertion(direction, lac.heads_in),
            })
        clearance = _valve_clearance_mm(spec, layout, entries, v_pt)
        cands.append({
            "entries": entries,
            "conflicts": conflicts,
            "dev": dev,
            "clearance": clearance,
            "shift": shift,
        })
    return cands


def _nearest_to_valve(spec, layout, entries: list[dict], v_pt, rim_r: float, limit: int = 2) -> list[dict]:
    """距阀孔最近的辐条（阀孔净空不可达时的冲突孔）。"""
    d = []
    for e in entries:
        a, b = _spoke_chord(spec, layout, e, rim_r)
        d.append((_pt_seg_dist(v_pt, a, b), e))
    d.sort(key=lambda t: (t[0], t[1]["rim_hole"], t[1]["hub_hole"]))
    return [
        {
            "rim_hole": e["rim_hole"],
            "side": e["side"],
            "hub_hole": e["hub_hole"],
            "reason": "valve_clearance",
            "clearance_mm": r3(dist),
        }
        for dist, e in d[:limit]
    ]


def _search_lacing(spec, layout, n_side: int):
    """按实际孔位搜索方向相位与孔位双射，返回 (entries, clearance, phase, shifts)。"""
    rim_r = spec.rim.erd_mm / 2.0
    v_ang = valve_angle(layout, spec.rim.valve_position)
    v_pt = (rim_r * math.cos(v_ang), rim_r * math.sin(v_ang))
    req = spec.valve_clearance_min_mm
    best = None      # 可行候选中的最优（净空最大）
    fallback = None  # 冲突最少（再比净空）的候选，用于无解时报错
    for phase in (0, 1):
        cands = {
            side: _side_phase_candidates(spec, layout, side, n_side, phase, v_pt, rim_r)
            for side in ("left", "right")
        }
        for cl in cands["left"]:
            for cr in cands["right"]:
                conflicts = cl["conflicts"] + cr["conflicts"]
                clearance = min(cl["clearance"], cr["clearance"])
                dev = cl["dev"] + cr["dev"]
                fkey = (len(conflicts), -clearance, dev, cl["shift"], cr["shift"], phase)
                if fallback is None or fkey < fallback[0]:
                    fallback = (fkey, cl, cr, conflicts, clearance)
                if conflicts or clearance < req - 1e-9:
                    continue
                key = (-clearance, dev, cl["shift"], cr["shift"], phase)
                if best is None or key < best[0]:
                    best = (key, cl, cr, phase, clearance)
    if best is None:
        _, cl, cr, conflicts, clearance = fallback
        if conflicts:
            raise WheelError(
                "LACING_INFEASIBLE",
                f"自动穿法无解：{len(conflicts)} 处孔位在实际角度下不满足交叉方向/交叉数设置",
                {
                    "conflicts": conflicts,
                    "cross": {"left": spec.left.cross, "right": spec.right.cross},
                },
            )
        raise WheelError(
            "LACING_INFEASIBLE",
            f"自动穿法无解：阀孔净空要求 {r3(req)} mm 不可达（最佳 {r3(clearance)} mm）",
            {
                "reason": "valve_clearance",
                "valve_clearance_min_mm": r3(req),
                "best_clearance_mm": r3(clearance),
                "conflicts": _nearest_to_valve(spec, layout, cl["entries"] + cr["entries"], v_pt, rim_r),
            },
        )
    _, cl, cr, phase, clearance = best
    entries = cl["entries"] + cr["entries"]
    shifts = {"left": cl["shift"], "right": cr["shift"]}
    return entries, clearance, phase, shifts


def _check_bijection(entries: list[dict], layout, n_side: int) -> None:
    """双射校验：圈孔/法兰孔重复时给出具体孔号与原因。"""
    seen_rim: dict[int, int] = {}
    for idx, e in enumerate(entries):
        i = e["rim_hole"]
        if i in seen_rim:
            raise WheelError(
                "DUPLICATE_HOLE_MAPPING",
                f"圈孔 {i} 被重复占用（第 {seen_rim[i]} 与第 {idx} 条映射），每个圈孔只能穿一根辐条",
                {"rim_hole": i, "entry_indexes": [seen_rim[i], idx]},
            )
        seen_rim[i] = idx
    seen_hub: dict[tuple, int] = {}
    for idx, e in enumerate(entries):
        key = (e["side"], e["hub_hole"])
        if key in seen_hub:
            raise WheelError(
                "DUPLICATE_HOLE_MAPPING",
                f"{e['side']} 侧法兰孔 {e['hub_hole']} 被重复占用（第 {seen_hub[key]} 与第 {idx} 条映射）",
                {"side": e["side"], "hub_hole": e["hub_hole"], "entry_indexes": [seen_hub[key], idx]},
            )
        seen_hub[key] = idx
    missing_rim = sorted(set(layout.rim_angles) - set(seen_rim))
    if missing_rim:
        raise WheelError(
            "MAPPING_INCOMPLETE",
            f"映射未覆盖全部圈孔，缺少圈孔 {missing_rim}",
            {"missing_rim_holes": missing_rim},
        )
    for side in ("left", "right"):
        used = {e["hub_hole"] for e in entries if e["side"] == side}
        missing_hub = sorted(set(range(n_side)) - used)
        if missing_hub:
            raise WheelError(
                "MAPPING_INCOMPLETE",
                f"{side} 侧法兰孔 {missing_hub} 未被使用",
                {"side": side, "missing_hub_holes": missing_hub},
            )


def _entries_from_override(spec, layout, n_side: int) -> list[dict]:
    entries = []
    for idx, m in enumerate(spec.mapping_override):
        if m.rim_hole not in layout.rim_angles:
            raise WheelError(
                "MAPPING_INVALID",
                f"第 {idx} 条映射的圈孔号 {m.rim_hole} 不存在于圈孔集合",
                {"entry_index": idx, "rim_hole": m.rim_hole},
            )
        if m.hub_hole >= n_side:
            raise WheelError(
                "MAPPING_INVALID",
                f"第 {idx} 条映射的法兰孔号 {m.hub_hole} 超出范围 0..{n_side - 1}",
                {"entry_index": idx, "hub_hole": m.hub_hole},
            )
        actual_side = layout.rim_sides[m.rim_hole]
        if actual_side != m.side:
            raise WheelError(
                "MAPPING_INVALID",
                f"第 {idx} 条映射：圈孔 {m.rim_hole} 在 {actual_side} 侧，不能穿到 {m.side} 法兰",
                {"entry_index": idx, "rim_hole": m.rim_hole, "side": m.side},
            )
        a_r = layout.rim_angles[m.rim_hole]
        a_h = layout.hub_angles[m.side][m.hub_hole]
        delta = _wrap_pi(a_h - a_r)
        if abs(delta) < 1e-9:
            direction = "radial"
        else:
            direction = "trailing" if delta > 0 else "leading"
        lac = getattr(spec, m.side)
        entries.append({
            "rim_hole": m.rim_hole,
            "side": m.side,
            "hub_hole": m.hub_hole,
            "direction": direction,
            "insertion": _insertion(direction, lac.heads_in),
        })
    return entries


def _build_sequence(entries: list[dict], first_rim_hole: int, layout) -> list[dict]:
    """编轮次序：含首根的组最先，其余按 内穿组 -> 外穿组、首根侧 -> 另一侧；
    组内从紧邻阀孔的圈孔开始按实际角度的旋转顺序排列，保证第 1 步即首根辐条。"""
    first_entry = next(e for e in entries if e["rim_hole"] == first_rim_hole)
    first_side = first_entry["side"]
    other_side = "left" if first_side == "right" else "right"
    groups = [(s, ins) for s in (first_side, other_side) for ins in ("heads_in", "heads_out")]
    first_group = (first_entry["side"], first_entry["insertion"])
    groups.sort(key=lambda g: 0 if g == first_group else 1)  # 稳定排序，首根所在组提到最前
    a0 = layout.rim_angles[first_rim_hole]
    seq = []
    step = 0
    for side, insertion in groups:
        group = [e for e in entries if e["side"] == side and e["insertion"] == insertion]
        group.sort(key=lambda e: (layout.rim_angles[e["rim_hole"]] - a0) % TWO_PI)
        for e in group:
            step += 1
            seq.append({
                "step": step,
                "side": side,
                "insertion": insertion,
                "direction": e["direction"],
                "rim_hole": e["rim_hole"],
                "hub_hole": e["hub_hole"],
            })
    return seq


def build_mapping(spec, layout=None) -> dict:
    """生成（或校验自定义）孔位映射，返回映射、首根定位、阀孔避让与编轮次序。"""
    if layout is None:
        layout = resolve_layout(spec)
    n_side = validate_counts(spec, layout)
    check_cross(spec, n_side)

    if spec.mapping_override is not None:
        entries = _entries_from_override(spec, layout, n_side)
        _check_bijection(entries, layout, n_side)
        phase = None
        shifts = None
        clearance = _valve_clearance_mm(spec, layout, entries)
    else:
        entries, clearance, phase, shifts = _search_lacing(spec, layout, n_side)
        _check_bijection(entries, layout, n_side)

    entries.sort(key=lambda e: e["rim_hole"])
    # 首根 = 阀孔顺时针侧紧邻的圈孔（按实际角度的角序取下一孔）
    first_rim_hole = next_rim_hole(layout, spec.rim.valve_position)
    first = next(e for e in entries if e["rim_hole"] == first_rim_hole)
    rim_r = spec.rim.erd_mm / 2.0

    return {
        "phase": phase,
        "flange_shift": shifts,
        "max_cross_per_side": max_cross(n_side),
        "first_spoke": {
            "rim_hole": first["rim_hole"],
            "hub_hole": first["hub_hole"],
            "side": first["side"],
            "direction": first["direction"],
            "insertion": first["insertion"],
            "note": f"首根辐条取阀孔顺时针侧第 {first_rim_hole} 号圈孔（按实际角度角序），相位已按阀孔避让选取",
        },
        "valve": {
            "position_between": [spec.rim.valve_position, first_rim_hole],
            "clearance_mm": r3(clearance),
            "clearance_deg": r3(math.degrees(clearance / rim_r)),
            "required_clearance_mm": r3(spec.valve_clearance_min_mm),
        },
        "mapping": entries,
        "sequence": _build_sequence(entries, first_rim_hole, layout),
        "crossing_note": "外穿（heads-out）辐条在最外侧交叉处压过内穿（heads-in）辐条",
    }
