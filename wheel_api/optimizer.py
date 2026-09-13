"""库存辐条 + 垫圈组合优化。

装配关系：垫圈厚度 t 使条帽座外移，等效理想长度变为 ideal + t，
故 偏差 err = 库存条长 - t - ideal。合格条件：
  |err| <= length_tolerance（允许长度误差）
  engagement = thread_length + min(0, err) >= min_thread_engagement（螺纹啮合）
  err <= max_protrusion（条帽顶端外露）
张力：T_left = ratio * T_right，两侧均须落在 [tension_min, tension_max]；
在可行区间内取中点使张力余量最大。
排序：最大长度偏差升序 -> 张力余量降序 -> 规格种数升序。
"""

from __future__ import annotations

from .geometry import r3


def _side_candidates(target: float, washer_options: list[float], spec, excluded: dict) -> list[dict]:
    cands = []
    for sp in spec.inventory:
        for t in washer_options:
            err = sp.length_mm - t - target
            if abs(err) > spec.length_tolerance_mm + 1e-9:
                excluded["length_tolerance"] += 1
                continue
            engagement = spec.spoke_thread_length_mm + min(0.0, err)
            if engagement < spec.min_thread_engagement_mm - 1e-9:
                excluded["thread_engagement"] += 1
                continue
            if err > spec.max_protrusion_mm + 1e-9:
                excluded["protrusion"] += 1
                continue
            cands.append({
                "spoke_length_mm": r3(sp.length_mm),
                "washer_mm": r3(t),
                "deviation_mm": r3(err),
                "thread_engagement_mm": r3(engagement),
                "_count": sp.count,
            })
    return cands


def optimize(geometry: dict, spec) -> dict:
    n_side = geometry["holes_per_side"]
    ratio = geometry["tension_ratio_left_to_right"]
    targets = {
        "left": geometry["sides"]["left"]["spoke_length_mm"],
        "right": geometry["sides"]["right"]["spoke_length_mm"],
    }

    # 张力可行区间（对右侧张力求解）
    tmin, tmax = spec.tension_min_n, spec.tension_max_n
    lo = max(tmin, tmin / ratio)
    hi = min(tmax, tmax / ratio)
    base = {
        "inputs": spec.model_dump(),
        "targets": {"left_ideal_mm": targets["left"], "right_ideal_mm": targets["right"]},
        "tension_ratio_left_to_right": ratio,
    }
    if lo > hi + 1e-9:
        base["feasible"] = False
        base["reason"] = (
            f"张力窗口不可行：张力比 {ratio} 要求 T_left = {r3(ratio)} x T_right，"
            f"在 [{r3(tmin)}, {r3(tmax)}] N 内无解"
        )
        base["combos"] = []
        return base

    t_right = (lo + hi) / 2.0
    t_left = t_right * ratio
    margin = min(t_right - tmin, tmax - t_right, t_left - tmin, tmax - t_left)
    tension_info = {
        "feasible_right_range_n": [r3(lo), r3(hi)],
        "chosen_right_n": r3(t_right),
        "left_n": r3(t_left),
        "margin_n": r3(margin),
    }

    washer_options = sorted({0.0} | {float(w) for w in spec.washers_mm})
    excluded = {"length_tolerance": 0, "thread_engagement": 0, "protrusion": 0, "inventory_count": 0}
    left_cands = _side_candidates(targets["left"], washer_options, spec, excluded)
    right_cands = _side_candidates(targets["right"], washer_options, spec, excluded)

    combos = []
    for lc in left_cands:
        for rc in right_cands:
            # 库存数量校验：同一长度在两侧共用时要累加需求
            need = {}
            for c, cnt in ((lc, n_side), (rc, n_side)):
                key = c["spoke_length_mm"]
                need[key] = need.get(key, 0) + cnt
            ok = True
            for c in (lc, rc):
                if c["_count"] is not None and need[c["spoke_length_mm"]] > c["_count"]:
                    ok = False
            if not ok:
                excluded["inventory_count"] += 1
                continue
            max_dev = max(abs(lc["deviation_mm"]), abs(rc["deviation_mm"]))
            specs = len({lc["spoke_length_mm"], rc["spoke_length_mm"]})
            combos.append({
                "left": {k: v for k, v in lc.items() if not k.startswith("_")},
                "right": {k: v for k, v in rc.items() if not k.startswith("_")},
                "max_deviation_mm": r3(max_dev),
                "tension_margin_n": tension_info["margin_n"],
                "spoke_spec_count": specs,
                "_sort": (max_dev, -margin, specs,
                          lc["spoke_length_mm"], rc["spoke_length_mm"],
                          lc["washer_mm"], rc["washer_mm"]),
            })

    combos.sort(key=lambda c: c["_sort"])
    out_combos = []
    for rank, c in enumerate(combos[: spec.limit], start=1):
        c = {k: v for k, v in c.items() if not k.startswith("_")}
        c["rank"] = rank
        out_combos.append(c)

    base.update({
        "feasible": True,
        "tension": tension_info,
        "washer_options_mm": washer_options,
        "combos": out_combos,
        "combos_total": len(combos),
        "excluded": excluded,
    })
    if not out_combos:
        base["reason"] = "无合格组合：库存长度/垫圈在允许误差与螺纹啮合要求下均不满足"
    return base
