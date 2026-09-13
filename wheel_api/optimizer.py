"""库存辐条 + 垫圈组合优化。

装配关系：垫圈厚度 t 使条帽座外移，等效理想长度变为 ideal + t。
偏差按**逐孔**理想长度判定（同一侧内/外穿修正使各孔理想长度略有散布）：
  err_hole = 库存条长 - t - ideal_hole
合格条件（对该侧全部孔取最不利值）：
  max|err_hole| <= length_tolerance（允许长度误差）
  engagement = thread_length + min(0, min err_hole) >= min_thread_engagement（螺纹啮合）
  max err_hole <= max_protrusion（条帽顶端外露）
库存数量：同一长度的多行库存先合并（数量累加，任一行不限量则不限量），
再与两侧需求量（各 n 根）比较。
张力：T_left = ratio * T_right，两侧均须落在 [tension_min, tension_max]；
在可行区间内取中点使张力余量最大。
排序：最大长度偏差升序 -> 张力余量降序 -> 规格种数升序。
"""

from __future__ import annotations

from .geometry import r3


def _merge_inventory(spec) -> list[tuple[float, "int | None"]]:
    """按长度合并库存行：数量累加；任一行 count 为 None 则该长度不限量。"""
    merged: dict[float, "int | None"] = {}
    for sp in spec.inventory:
        length = float(sp.length_mm)
        if length not in merged:
            merged[length] = sp.count
        elif merged[length] is not None:
            merged[length] = None if sp.count is None else merged[length] + sp.count
    return sorted(merged.items())


def _side_candidates(inventory, ideals: list[float], washer_options: list[float],
                     spec, excluded: dict) -> list[dict]:
    """单侧候选：对每个（长度, 垫圈）按该侧全部孔的逐孔偏差判定。"""
    cands = []
    for length, count in inventory:
        for t in washer_options:
            errs = [length - t - ideal for ideal in ideals]
            max_abs = max(abs(e) for e in errs)
            if max_abs > spec.length_tolerance_mm + 1e-9:
                excluded["length_tolerance"] += 1
                continue
            # 螺纹啮合按最不利（相对最短的孔）判定
            engagement = spec.spoke_thread_length_mm + min(0.0, min(errs))
            if engagement < spec.min_thread_engagement_mm - 1e-9:
                excluded["thread_engagement"] += 1
                continue
            if max(errs) > spec.max_protrusion_mm + 1e-9:
                excluded["protrusion"] += 1
                continue
            cands.append({
                "spoke_length_mm": r3(length),
                "washer_mm": r3(t),
                "deviation_mm": r3(max_abs),
                "deviation_range_mm": [r3(min(errs)), r3(max(errs))],
                "thread_engagement_mm": r3(engagement),
                "_count": count,
            })
    return cands


def optimize(geometry: dict, spec) -> dict:
    n_side = geometry["holes_per_side"]
    ratio = geometry["tension_ratio_left_to_right"]
    per_hole = geometry["per_hole"]
    ideals = {
        side: [h["length_mm"] for h in per_hole if h["side"] == side]
        for side in ("left", "right")
    }
    targets = {
        side: {
            "nominal_mm": geometry["sides"][side]["spoke_length_mm"],
            "per_hole_min_mm": r3(min(ideals[side])),
            "per_hole_max_mm": r3(max(ideals[side])),
        }
        for side in ("left", "right")
    }

    # 张力可行区间（对右侧张力求解）
    tmin, tmax = spec.tension_min_n, spec.tension_max_n
    lo = max(tmin, tmin / ratio)
    hi = min(tmax, tmax / ratio)
    base = {
        "inputs": spec.model_dump(),
        "targets": targets,
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

    inventory = _merge_inventory(spec)
    washer_options = sorted({0.0} | {float(w) for w in spec.washers_mm})
    excluded = {"length_tolerance": 0, "thread_engagement": 0, "protrusion": 0, "inventory_count": 0}
    left_cands = _side_candidates(inventory, ideals["left"], washer_options, spec, excluded)
    right_cands = _side_candidates(inventory, ideals["right"], washer_options, spec, excluded)

    combos = []
    for lc in left_cands:
        for rc in right_cands:
            # 库存数量校验：同一长度在两侧共用时要累加需求（每侧 n 根）
            need = {}
            for c in (lc, rc):
                key = c["spoke_length_mm"]
                need[key] = need.get(key, 0) + n_side
            ok = True
            for c in (lc, rc):
                if c["_count"] is not None and need[c["spoke_length_mm"]] > c["_count"]:
                    ok = False
            if not ok:
                excluded["inventory_count"] += 1
                continue
            max_dev = max(lc["deviation_mm"], rc["deviation_mm"])
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
