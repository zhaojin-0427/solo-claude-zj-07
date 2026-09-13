"""孔位映射、阀孔避让、首根定位与编轮次序。

相位选择：对每种相位（phase 0/1，即顺/逆向组从哪个圈孔开始），
计算阀孔到最近辐条弦的距离，取避让更好的一种；相位不改变辐条长度
（|delta| = 2*pi*cross/n 不变），只改变阀孔两侧辐条的收拢/发散关系。
"""

from __future__ import annotations

import math

from .errors import WheelError
from .geometry import (
    check_cross,
    hub_angle,
    max_cross,
    r3,
    rim_angle,
    rim_side,
    validate_counts,
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


def _generate_entries(spec, n_side: int, phase: int) -> list[dict]:
    """标准映射：圈孔 2m/2m+1 -> 法兰孔 (m +/- cross) mod n。

    phase 翻转顺/逆向组的起始奇偶，用于阀孔避让；两种相位长度一致。
    """
    entries = []
    for side in ("right", "left"):
        lac = getattr(spec, side)
        k = lac.cross
        for m in range(n_side):
            i = 2 * m if side == "right" else 2 * m + 1
            direction = "trailing" if (m + phase) % 2 == 0 else "leading"
            j = (m + k) % n_side if direction == "trailing" else (m - k) % n_side
            entries.append({
                "rim_hole": i,
                "side": side,
                "hub_hole": j,
                "direction": direction if k > 0 else "radial",
                "insertion": _insertion(direction if k > 0 else "radial", lac.heads_in),
            })
    return entries


def _valve_clearance_mm(spec, entries: list[dict], n_side: int) -> float:
    n_total = spec.rim.holes
    rim_r = spec.rim.erd_mm / 2.0
    v_ang = (spec.rim.valve_position + 0.5) * 2.0 * math.pi / n_total
    v_pt = (rim_r * math.cos(v_ang), rim_r * math.sin(v_ang))
    best = math.inf
    for e in entries:
        side = e["side"]
        r = (spec.hub.flange_pcd_right_mm if side == "right" else spec.hub.flange_pcd_left_mm) / 2.0
        a_r = rim_angle(e["rim_hole"], n_total)
        a_h = hub_angle(side, e["hub_hole"], n_side)
        a_pt = (rim_r * math.cos(a_r), rim_r * math.sin(a_r))
        b_pt = (r * math.cos(a_h), r * math.sin(a_h))
        best = min(best, _pt_seg_dist(v_pt, a_pt, b_pt))
    return best


def _check_bijection(entries: list[dict], n_side: int, n_total: int) -> None:
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
    missing_rim = sorted(set(range(n_total)) - set(seen_rim))
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


def _entries_from_override(spec, n_side: int) -> list[dict]:
    n_total = spec.rim.holes
    entries = []
    for idx, m in enumerate(spec.mapping_override):
        if m.rim_hole >= n_total:
            raise WheelError(
                "MAPPING_INVALID",
                f"第 {idx} 条映射的圈孔号 {m.rim_hole} 超出范围 0..{n_total - 1}",
                {"entry_index": idx, "rim_hole": m.rim_hole},
            )
        if m.hub_hole >= n_side:
            raise WheelError(
                "MAPPING_INVALID",
                f"第 {idx} 条映射的法兰孔号 {m.hub_hole} 超出范围 0..{n_side - 1}",
                {"entry_index": idx, "hub_hole": m.hub_hole},
            )
        if rim_side(m.rim_hole) != m.side:
            raise WheelError(
                "MAPPING_INVALID",
                f"第 {idx} 条映射：圈孔 {m.rim_hole} 在 {rim_side(m.rim_hole)} 侧，不能穿到 {m.side} 法兰",
                {"entry_index": idx, "rim_hole": m.rim_hole, "side": m.side},
            )
        a_r = rim_angle(m.rim_hole, n_total)
        a_h = hub_angle(m.side, m.hub_hole, n_side)
        delta = math.atan2(math.sin(a_h - a_r), math.cos(a_h - a_r))
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


def _build_sequence(entries: list[dict], first_rim_hole: int, n_total: int) -> list[dict]:
    """编轮次序：含首根的组最先，其余按 内穿组 -> 外穿组、首根侧 -> 另一侧；
    组内从紧邻阀孔的圈孔开始按旋转顺序排列，保证第 1 步即首根辐条。"""
    first_entry = next(e for e in entries if e["rim_hole"] == first_rim_hole)
    first_side = first_entry["side"]
    other_side = "left" if first_side == "right" else "right"
    groups = [(s, ins) for s in (first_side, other_side) for ins in ("heads_in", "heads_out")]
    first_group = (first_entry["side"], first_entry["insertion"])
    groups.sort(key=lambda g: 0 if g == first_group else 1)  # 稳定排序，首根所在组提到最前
    seq = []
    step = 0
    for side, insertion in groups:
        group = [e for e in entries if e["side"] == side and e["insertion"] == insertion]
        group.sort(key=lambda e: (e["rim_hole"] - first_rim_hole) % n_total)
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


def build_mapping(spec) -> dict:
    """生成（或校验自定义）孔位映射，返回映射、首根定位、阀孔避让与编轮次序。"""
    n_side = validate_counts(spec)
    check_cross(spec, n_side)
    n_total = spec.rim.holes

    if spec.mapping_override is not None:
        entries = _entries_from_override(spec, n_side)
        _check_bijection(entries, n_side, n_total)
        phase = None
        clearance = _valve_clearance_mm(spec, entries, n_side)
    else:
        best = None
        for phase_candidate in (0, 1):
            cand = _generate_entries(spec, n_side, phase_candidate)
            c = _valve_clearance_mm(spec, cand, n_side)
            if best is None or c > best[1] + 1e-9:
                best = (cand, c, phase_candidate)
        entries, clearance, phase = best
        _check_bijection(entries, n_side, n_total)

    entries.sort(key=lambda e: e["rim_hole"])
    # 首根 = 阀孔顺时针侧紧邻的圈孔（阀孔在 valve_position 与下一孔之间）
    first_rim_hole = (spec.rim.valve_position + 1) % n_total
    first = next(e for e in entries if e["rim_hole"] == first_rim_hole)
    rim_r = spec.rim.erd_mm / 2.0

    return {
        "phase": phase,
        "max_cross_per_side": max_cross(n_side),
        "first_spoke": {
            "rim_hole": first["rim_hole"],
            "hub_hole": first["hub_hole"],
            "side": first["side"],
            "direction": first["direction"],
            "insertion": first["insertion"],
            "note": f"首根辐条取阀孔顺时针侧第 {first_rim_hole} 号圈孔，相位已按阀孔避让选取",
        },
        "valve": {
            "position_between": [spec.rim.valve_position, (spec.rim.valve_position + 1) % n_total],
            "clearance_mm": r3(clearance),
            "clearance_deg": r3(math.degrees(clearance / rim_r)),
        },
        "mapping": entries,
        "sequence": _build_sequence(entries, first_rim_hole, n_total),
        "crossing_note": "外穿（heads-out）辐条在最外侧交叉处压过内穿（heads-in）辐条",
    }
