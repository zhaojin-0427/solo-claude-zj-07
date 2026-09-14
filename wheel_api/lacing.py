"""孔位映射、阀孔避让、首根定位与编轮次序。

自动穿法按**实际孔位**回溯搜索孔位双射（不限于整体循环移位）：
逐角序圈孔分配法兰孔，约束均按实际几何判定——
- 交叉方向：同向（同为顺/逆向）辐条的弦不得相交；
- 交叉数：每根辐条与反向辐条的实际弦交叉数必须等于设置值
  （k=0 径向时任何交叉都不可）；
- k>0 时顺/逆向辐条数量平衡（各半）；
- 阀孔净空：阀孔到最近辐条弦的距离 >= valve_clearance_min_mm。
在全部合法双射中取阀孔净空最大者（并列时取夹角散布最小、搜索序最先者）；
无解时抛出 LACING_INFEASIBLE 并给出冲突孔（按循环移位候选中冲突最少者
的实际几何交叉诊断）。
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

# 回溯搜索上限：防止病态孔位下搜索失控
_MAX_SEARCH_NODES = 300_000
_MAX_SEARCH_NODES_FALLBACK = 100_000
_MAX_SOLUTIONS_PER_SIDE = 300


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


def _orient(a, b, c) -> float:
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def _segments_cross(p1, p2, p3, p4) -> bool:
    """两弦严格相交（双射保证端点各异，不会共享端点）。"""
    d1 = _orient(p3, p4, p1)
    d2 = _orient(p3, p4, p2)
    d3 = _orient(p1, p2, p3)
    d4 = _orient(p1, p2, p4)
    return d1 * d2 < 0.0 and d3 * d4 < 0.0


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


def _cross_masks(rim_pts: list, hub_pts: list, n: int) -> list:
    """预计算交叉位掩码：mask[m][j][m2] 的第 j2 位 = (m->j) 与 (m2->j2) 是否交叉。

    交叉关系对称，先算 m2 > m 的上三角再镜像，避免 O(n^4) 重复计算。
    """
    masks = [[[0] * n for _ in range(n)] for _ in range(n)]
    for m in range(n):
        p1 = rim_pts[m]
        for j in range(n):
            p2 = hub_pts[j]
            row = masks[m][j]
            for m2 in range(m + 1, n):
                q1 = rim_pts[m2]
                bits = 0
                for j2 in range(n):
                    if _segments_cross(p1, p2, q1, hub_pts[j2]):
                        bits |= 1 << j2
                row[m2] = bits
    for m in range(n):
        for j in range(n):
            for m2 in range(m + 1, n):
                bits = masks[m][j][m2]
                if not bits:
                    continue
                for j2 in range(n):
                    if (bits >> j2) & 1:
                        masks[m2][j2][m] |= 1 << j
    return masks


def _side_assignments(layout, side: str, n_side: int, k: int,
                      rim_r: float, flange_r: float):
    """回溯搜索该侧全部合法孔位双射。

    返回 (圈孔角序, [assign])；assign[m] = 角序第 m 个圈孔分配的法兰孔号。
    约束：同向辐条弦不相交、每根辐条与反向辐条的实际交叉数 == k、
    k>0 时顺/逆向各 n/2 根。候选按 |delta| 接近 k 倍节距排序，先找到紧凑解。
    两阶段：先在 |delta| ∈ k·节距 ± 半节距 的紧凑窗口内搜索（覆盖常规轮组），
    无解再在完整域上搜索（非等距孔位下合法解可能跨度更大）。
    """
    order = [i for i in layout.rim_order if layout.rim_sides[i] == side]
    rim_ang = [layout.rim_angles[i] for i in order]
    rim_pts = [(rim_r * math.cos(a), rim_r * math.sin(a)) for a in rim_ang]
    hub_ang = layout.hub_angles[side]
    hub_pts = [(flange_r * math.cos(a), flange_r * math.sin(a)) for a in hub_ang]
    pitch = TWO_PI / n_side
    target = k * pitch
    half = n_side // 2
    deltas = [[_wrap_pi(hub_ang[j] - rim_ang[m]) for j in range(n_side)] for m in range(n_side)]
    masks = _cross_masks(rim_pts, hub_pts, n_side)
    solutions: list[tuple] = []

    def backtrack(window, node_cap: int) -> None:
        assign = [-1] * n_side
        used = [False] * n_side
        dirs = [0] * n_side
        counts = [0] * n_side
        nodes = [0]

        def rec(m: int, n_trail: int) -> None:
            if len(solutions) >= _MAX_SOLUTIONS_PER_SIDE or nodes[0] > node_cap:
                return
            if m == n_side:
                if all(c == k for c in counts):
                    solutions.append(tuple(assign))
                return
            nodes[0] += 1
            row = deltas[m]
            cands = []
            for j in range(n_side):
                if used[j]:
                    continue
                delta = row[j]
                if k > 0 and abs(delta) < 1e-9:
                    continue
                if window is not None and abs(abs(delta) - target) > window:
                    continue
                cands.append((abs(abs(delta) - target), j, delta))
            cands.sort()
            remaining = n_side - m - 1  # 当前孔之后尚未分配的圈孔数
            mask_row = masks[m]
            for _, j, delta in cands:
                d = 0 if abs(delta) < 1e-9 else (1 if delta > 0 else -1)
                n_t = n_trail + (1 if d > 0 else 0)
                if k > 0 and (n_t > half or (m + 1) - n_t > half):
                    continue
                crossed = []
                ok = True
                mj = mask_row[j]
                for m2 in range(m):
                    if not (mj[m2] >> assign[m2]) & 1:
                        continue
                    if d != 0 and dirs[m2] == d:
                        ok = False  # 同向辐条交叉
                        break
                    if counts[m2] + 1 > k:
                        ok = False  # 对方交叉数超限
                        break
                    crossed.append(m2)
                if not ok or len(crossed) > k:
                    continue
                # 当前辐条自身最终交叉数不可能达到 k 时剪枝
                if len(crossed) + remaining < k:
                    continue
                # 已分配辐条（含当前辐条带来的交叉）最终交叉数不可能达到 k 时剪枝
                cset = set(crossed)
                if any(counts[m2] + (1 if m2 in cset else 0) + remaining < k
                       for m2 in range(m)):
                    continue
                assign[m] = j
                used[j] = True
                dirs[m] = d
                counts[m] = len(crossed)
                for m2 in crossed:
                    counts[m2] += 1
                rec(m + 1, n_t)
                for m2 in crossed:
                    counts[m2] -= 1
                counts[m] = 0
                dirs[m] = 0
                used[j] = False
                assign[m] = -1

        rec(0, 0)

    # 阶段一：紧凑窗口（常规轮组在此完成，含等距与典型成对钻孔）
    backtrack(0.5 * pitch, _MAX_SEARCH_NODES)
    # 阶段二：完整域（非等距孔位下合法解可能超出窗口）
    if not solutions:
        backtrack(None, _MAX_SEARCH_NODES_FALLBACK)
    return order, solutions


def _solution_entries(layout, side: str, order: list[int], assign: tuple, lac, k: int):
    """由双射生成映射条目与夹角散布 dev。"""
    hub_ang = layout.hub_angles[side]
    pitch = TWO_PI / len(order)
    entries = []
    dev = 0.0
    for m, rim_id in enumerate(order):
        j = assign[m]
        delta = _wrap_pi(hub_ang[j] - layout.rim_angles[rim_id])
        if k > 0:
            direction = "trailing" if delta > 0 else "leading"
        else:
            direction = "radial" if abs(delta) < 1e-9 else ("trailing" if delta > 0 else "leading")
        dev += abs(abs(delta) - k * pitch)
        entries.append({
            "rim_hole": rim_id,
            "side": side,
            "hub_hole": j,
            "direction": direction,
            "insertion": _insertion(direction, lac.heads_in),
        })
    return entries, dev


def _cyclic_assignments(layout, side: str, n_side: int, k: int):
    """旧式整体循环移位候选（仅用于无解时的冲突诊断）。"""
    order = [i for i in layout.rim_order if layout.rim_sides[i] == side]
    h_order = sorted(range(n_side), key=lambda j: (layout.hub_angles[side][j], j))
    for phase in (0, 1):
        for shift in range(n_side):
            assign = []
            for m in range(n_side):
                trailing = (m + phase) % 2 == 0
                hj = (m + shift + k) % n_side if (trailing or k == 0) else (m + shift - k) % n_side
                assign.append(h_order[hj])
            yield phase, shift, order, assign


def _diagnose_side(layout, side: str, n_side: int, k: int,
                   rim_r: float, flange_r: float) -> list[dict]:
    """无解侧诊断：在循环移位候选中按实际几何交叉找逐孔冲突最少者。"""
    best = None
    for phase, shift, order, assign in _cyclic_assignments(layout, side, n_side, k):
        rim_pts = [(rim_r * math.cos(layout.rim_angles[i]), rim_r * math.sin(layout.rim_angles[i]))
                   for i in order]
        hub_pts = [(flange_r * math.cos(layout.hub_angles[side][j]),
                    flange_r * math.sin(layout.hub_angles[side][j])) for j in range(n_side)]
        dirs = []
        for m in range(n_side):
            delta = _wrap_pi(layout.hub_angles[side][assign[m]] - layout.rim_angles[order[m]])
            dirs.append(0 if abs(delta) < 1e-9 else (1 if delta > 0 else -1))
        counts = [0] * n_side
        bad_same = [False] * n_side
        for a in range(n_side):
            for b in range(a + 1, n_side):
                if not _segments_cross(rim_pts[a], hub_pts[assign[a]], rim_pts[b], hub_pts[assign[b]]):
                    continue
                if dirs[a] != 0 and dirs[a] == dirs[b]:
                    bad_same[a] = bad_same[b] = True
                else:
                    counts[a] += 1
                    counts[b] += 1
        conflicts = []
        for m in range(n_side):
            rim_id = order[m]
            j = assign[m]
            delta = _wrap_pi(layout.hub_angles[side][j] - layout.rim_angles[rim_id])
            if bad_same[m]:
                conflicts.append({
                    "rim_hole": rim_id,
                    "side": side,
                    "hub_hole": j,
                    "reason": "cross_direction",
                    "delta_deg": r3(math.degrees(delta)),
                })
            elif counts[m] != k:
                conflicts.append({
                    "rim_hole": rim_id,
                    "side": side,
                    "hub_hole": j,
                    "reason": "cross_count",
                    "cross_requested": k,
                    "cross_effective": counts[m],
                    "delta_deg": r3(math.degrees(delta)),
                })
        key = (len(conflicts), phase, shift)
        if best is None or key < best[0]:
            best = (key, conflicts)
    return best[1]


def _cyclic_form(order: list[int], assign: tuple, layout, side: str, n_side: int, k: int):
    """若解等价于某 (phase, shift) 的整体循环移位，返回 (phase, shift)，否则 None。"""
    h_order = sorted(range(n_side), key=lambda j: (layout.hub_angles[side][j], j))
    pos = {j: r for r, j in enumerate(h_order)}
    ranks = [pos[j] for j in assign]
    for phase in (0, 1):
        for shift in range(n_side):
            ok = True
            for m in range(n_side):
                trailing = (m + phase) % 2 == 0
                hj = (m + shift + k) % n_side if (trailing or k == 0) else (m + shift - k) % n_side
                if ranks[m] != hj:
                    ok = False
                    break
            if ok:
                return phase, shift
    return None


def _derive_phase_shift(cl: dict, cr: dict, layout, n_side: int, spec):
    """由最终双射反推 phase / flange_shift（非循环解的侧为 None）。"""
    forms = {}
    for side, c in (("left", cl), ("right", cr)):
        forms[side] = _cyclic_form(c["order"], c["assign"], layout, side, n_side,
                                   getattr(spec, side).cross)
    if forms["right"] is not None:
        phase = forms["right"][0]
    else:
        rank0 = next(e for e in cr["entries"] if e["rim_hole"] == cr["order"][0])
        phase = 0 if rank0["direction"] == "trailing" else 1
    shifts = {side: (forms[side][1] if forms[side] is not None else None)
              for side in ("left", "right")}
    return phase, shifts


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
    """按实际孔位搜索孔位双射，返回 (entries, clearance, phase, shifts)。"""
    rim_r = spec.rim.erd_mm / 2.0
    v_ang = valve_angle(layout, spec.rim.valve_position)
    v_pt = (rim_r * math.cos(v_ang), rim_r * math.sin(v_ang))
    req = spec.valve_clearance_min_mm
    per_side = {}
    for side in ("left", "right"):
        lac = getattr(spec, side)
        flange_r = (spec.hub.flange_pcd_left_mm if side == "left"
                    else spec.hub.flange_pcd_right_mm) / 2.0
        order, solutions = _side_assignments(layout, side, n_side, lac.cross, rim_r, flange_r)
        if not solutions:
            conflicts = _diagnose_side(layout, side, n_side, lac.cross, rim_r, flange_r)
            raise WheelError(
                "LACING_INFEASIBLE",
                f"自动穿法无解：{side} 侧在实际孔位下不存在满足交叉方向/交叉数设置的孔位双射",
                {
                    "conflicts": conflicts,
                    "side": side,
                    "cross": {"left": spec.left.cross, "right": spec.right.cross},
                },
            )
        evaluated = []
        for idx, assign in enumerate(solutions):
            entries, dev = _solution_entries(layout, side, order, assign, lac, lac.cross)
            clearance = _valve_clearance_mm(spec, layout, entries, v_pt)
            evaluated.append({
                "entries": entries, "dev": dev, "clearance": clearance,
                "assign": assign, "order": order, "idx": idx,
            })
        per_side[side] = evaluated

    best = None
    for cl in per_side["left"]:
        for cr in per_side["right"]:
            clearance = min(cl["clearance"], cr["clearance"])
            key = (-clearance, cl["dev"] + cr["dev"], cl["idx"], cr["idx"])
            if best is None or key < best[0]:
                best = (key, cl, cr, clearance)
    _, cl, cr, clearance = best
    if clearance < req - 1e-9:
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
    phase, shifts = _derive_phase_shift(cl, cr, layout, n_side, spec)
    return cl["entries"] + cr["entries"], clearance, phase, shifts


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
