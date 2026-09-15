"""服役载荷校核单路由：绑定不可变轮组版本，草拟 -> 已采用。

- POST /plans/{plan_id}/load-sheets         以不可变方案版本（默认最新，?version=N）创建草拟校核单
- GET  /load-sheets                         列出校核单（可按 ?plan_id= 过滤）
- GET  /load-sheets/{sheet_id}              读取校核单（已采用者冻结，重复读取逐字节一致）
- POST /load-sheets/{sheet_id}/revisions    以同一轮组版本生成修订草拟（来源/工况可改）
- POST /load-sheets/{sheet_id}/adopt        采用（冻结；可改用搜索结果中的方案编号）
- DELETE /load-sheets/{sheet_id}            放弃草拟（已采用不可删）

校核在写入前完成：方程无解或平衡残差超限直接返回 LOAD_CASE_INFEASIBLE 并
指出工况角度，不留下校核单记录。采用后文档以规范化 JSON 冻结在 SQLite 中，
相同已采用版本重复计算/读取一致；预张力方案只是建议，不写回调校批次。
"""

from __future__ import annotations

import json
from datetime import datetime, timezone

from fastapi import APIRouter
from fastapi.responses import Response

from . import service_load, storage
from .errors import WheelError
from .schemas import AdoptRequest, LoadSheetCreate, LoadSheetRevision

router = APIRouter()


def _now() -> str:
    return datetime.now(timezone.utc).isoformat()


def _json_text(payload: dict, status_code: int = 200) -> Response:
    return Response(content=storage.canonical(payload),
                    media_type="application/json", status_code=status_code)


def _plan_snapshot(plan_id: str, version: int | None) -> tuple[dict, int]:
    row = storage.plan_row(plan_id)
    if row is None:
        raise WheelError("PLAN_NOT_FOUND", f"方案 {plan_id} 不存在",
                         {"plan_id": plan_id}, status=404)
    ver = row[3] if version is None else version
    text = storage.get_snapshot(plan_id, ver)
    if text is None:
        raise WheelError(
            "VERSION_NOT_FOUND",
            f"方案 {plan_id} 的版本 {ver} 不存在",
            {"plan_id": plan_id, "version": ver}, status=404,
        )
    return json.loads(text), ver


def _load(sheet_id: str) -> tuple[dict, str, int]:
    row = storage.load_sheet_row(sheet_id)
    if row is None:
        raise WheelError("LOAD_SHEET_NOT_FOUND",
                         f"服役载荷校核单 {sheet_id} 不存在",
                         {"load_sheet_id": sheet_id}, status=404)
    return json.loads(row[4]), row[1], row[2]


def _require_draft(doc: dict, action: str) -> None:
    if doc["status"] != "draft":
        raise WheelError(
            "LOAD_SHEET_STATUS_INVALID",
            f"校核单当前状态为 {doc['status']}，不能{action}（仅草拟状态允许）",
            {"load_sheet_id": doc["load_sheet_id"], "status": doc["status"],
             "allowed": ["draft"], "action": action},
        )


# ---------------------------------------------------------------------------
# 预张力来源
# ---------------------------------------------------------------------------

def _resolve_pretension(source, plan_id: str, ver: int,
                        snapshot: dict) -> tuple[list[float], dict]:
    """返回 (逐孔预张力（按 per_hole 孔序）, 来源说明)。"""
    geo = snapshot["geometry"]
    ordered_holes = sorted(h["rim_hole"] for h in geo["per_hole"])
    side_of = {h["rim_hole"]: h["side"] for h in geo["per_hole"]}

    if source.type == "uniform":
        values = [
            float(source.right_n if side_of[h] == "right" else source.left_n)
            for h in ordered_holes
        ]
        info = {
            "type": "uniform",
            "left_n": float(source.left_n),
            "right_n": float(source.right_n),
        }
        return values, info

    # finalized_batch：取最后一轮已冻结测量中的逐条实测张力
    row = storage.batch_row(source.batch_id)
    if row is None:
        raise WheelError(
            "TENSION_SOURCE_INVALID",
            f"调校批次 {source.batch_id} 不存在，不能作为实测张力来源",
            {"batch_id": source.batch_id}, status=404,
        )
    bstate = json.loads(row[3])
    if bstate["status"] != "finalized":
        raise WheelError(
            "TENSION_SOURCE_INVALID",
            f"调校批次 {source.batch_id} 状态为 {bstate['status']}，"
            "只有已定稿（finalized）批次可作为实测张力来源",
            {"batch_id": source.batch_id, "status": bstate["status"]},
        )
    if row[1] != plan_id or row[2] != ver:
        raise WheelError(
            "TENSION_SOURCE_INVALID",
            f"调校批次 {source.batch_id} 绑定方案 {row[1]} 版本 {row[2]}，"
            f"与校核单来源方案 {plan_id} 版本 {ver} 不一致",
            {"batch_id": source.batch_id,
             "batch_plan_ref": {"plan_id": row[1], "version": row[2]},
             "sheet_plan_ref": {"plan_id": plan_id, "version": ver}},
        )
    rounds = bstate.get("rounds", [])
    if not rounds:
        raise WheelError(
            "TENSION_SOURCE_INVALID",
            f"调校批次 {source.batch_id} 没有任何已确认测量轮，无法取实测张力",
            {"batch_id": source.batch_id},
        )
    final_round = rounds[-1]
    points = {p["rim_hole"]: float(p["tension_n"])
              for p in final_round["measurement"]["points"]}
    missing = [h for h in ordered_holes if h not in points]
    if missing:
        raise WheelError(
            "TENSION_SOURCE_INVALID",
            f"调校批次 {source.batch_id} 末轮测量缺少 {len(missing)} 个圈孔的张力",
            {"batch_id": source.batch_id, "missing_rim_holes": missing},
        )
    values = [points[h] for h in ordered_holes]
    nonpositive = [ordered_holes[i] for i, t in enumerate(values) if t <= 0.0]
    if nonpositive:
        raise WheelError(
            "TENSION_SOURCE_INVALID",
            f"调校批次 {source.batch_id} 中存在非正实测张力，无法建立预张力模型",
            {"batch_id": source.batch_id, "rim_holes": nonpositive[:20]},
        )
    info = {
        "type": "finalized_batch",
        "batch_id": source.batch_id,
        "plan_ref": {"plan_id": row[1], "version": row[2]},
        "finalized_at": bstate.get("finalized_at"),
        "sampled_round": final_round["round"],
        "sampled_at": final_round.get("confirmed_at"),
        "note": "取该批次最后一轮已冻结测量的逐条实测张力（不写回批次）",
    }
    return values, info


def _limit_arrays(spec, snapshot: dict) -> tuple[list[float], list[float]]:
    ov = spec.tension_limits_override
    tmin, tmax = [], []
    for h in sorted(snapshot["geometry"]["per_hole"], key=lambda x: x["rim_hole"]):
        side_ov = getattr(ov, h["side"], None) if ov else None
        tmin.append(float(side_ov.min_n if side_ov else spec.tension_min_n))
        tmax.append(float(side_ov.max_n if side_ov else spec.tension_max_n))
    return tmin, tmax


def _factored_loads(spec) -> dict:
    loads = spec.loads
    f = float(loads.load_factor)
    torque_nmm = float(loads.torque_nm) * 1000.0 * f
    if loads.torque_direction == "brake":
        torque_nmm = -torque_nmm
    return {
        "radial_n": float(loads.radial_n) * f,
        "lateral_n": float(loads.lateral_n) * f,
        "torque_nmm": torque_nmm,
    }


def _case_doc(spec, angles: list[float]) -> dict:
    loads = spec.loads
    return {
        "angles": {"start_deg": float(spec.sweep.start_deg),
                   "end_deg": float(spec.sweep.end_deg),
                   "step_deg": float(spec.sweep.step_deg),
                   "angle_count": len(angles),
                   "angles_deg": [round(float(a), 6) for a in angles]},
        "loads": {
            "radial_n": float(loads.radial_n),
            "lateral_n": float(loads.lateral_n),
            "torque_nm": float(loads.torque_nm),
            "torque_direction": loads.torque_direction,
            "load_factor": float(loads.load_factor),
            "units": {"radial_n": "N", "lateral_n": "N",
                      "torque_nm": "N·m（求解换算为 N·mm）",
                      "angles": "deg（度，0° = 0 号孔方向，顺时针递增）",
                      "tension": "N", "displacement": "mm"},
        },
        "factored_loads": _factored_loads(spec),
        "residual_tolerance": float(spec.residual_tolerance),
        "tension_min_n": float(spec.tension_min_n),
        "tension_max_n": float(spec.tension_max_n),
        "tension_limits_override": (
            spec.tension_limits_override.model_dump()
            if spec.tension_limits_override is not None else None
        ),
    }


def _search_spec(spec) -> dict | None:
    ps = spec.pretension_search
    if ps is None:
        return None
    return {
        "locked": set(ps.locked_rim_holes),
        "max_adjustment_n": float(ps.max_adjustment_n),
        "step_n": float(ps.step_n),
        "limit": ps.limit,
        "selected_index": ps.selected_index,
    }


def _build_doc(sheet_id: str, name: str, plan_id: str, ver: int,
               snapshot: dict, spec, status: str,
               created_at: str, revises: str | None) -> dict:
    """全部校验与计算在此完成；失败抛出，不写库（调用方保证原子性）。"""
    model = service_load.build_spoke_model(snapshot)
    hole_set = {s["rim_hole"] for s in model["spokes"]}
    search = _search_spec(spec)
    if search is not None:
        bad = sorted(search["locked"] - hole_set)
        if bad:
            raise WheelError(
                "LOAD_SHEET_HOLE_UNKNOWN",
                f"锁定圈孔不属于来源方案版本: {bad}",
                {"rim_holes": bad,
                 "valid_note": "以绑定方案版本的圈孔编号为准"},
            )
    pretension, source_info = _resolve_pretension(
        spec.tension_source, plan_id, ver, snapshot)
    tmin, tmax = _limit_arrays(spec, snapshot)
    angles = spec.sweep.angles()
    case = _case_doc(spec, angles)
    result = service_load.build_check_result(
        model, pretension, tmin, tmax, angles,
        case["factored_loads"], float(spec.residual_tolerance), search)
    ps = spec.pretension_search
    return {
        "load_sheet_id": sheet_id,
        "name": name,
        "status": status,
        "plan_ref": {"plan_id": plan_id, "version": ver,
                     "formula_version": snapshot["formula_version"]},
        "service_formula_version": service_load.SERVICE_FORMULA_VERSION,
        "service_formulas": service_load.SERVICE_FORMULAS,
        "tension_source": source_info,
        "case": case,
        "pretension": [
            {"rim_hole": s["rim_hole"], "side": s["side"],
             "pretension_n": round(pretension[i], 3)}
            for i, s in enumerate(model["spokes"])
        ],
        "pretension_search_request": (
            None if ps is None
            else {"locked_rim_holes": sorted(set(ps.locked_rim_holes)),
                  "max_adjustment_n": float(ps.max_adjustment_n),
                  "step_n": float(ps.step_n), "limit": ps.limit,
                  "selected_index": ps.selected_index}
        ),
        "result": result,
        "revises": revises,
        "adopted": None,
        "created_at": created_at,
        "updated_at": created_at,
    }


# ---------------------------------------------------------------------------
# 创建 / 列出 / 读取 / 修订
# ---------------------------------------------------------------------------

@router.post("/plans/{plan_id}/load-sheets", status_code=201)
def create_load_sheet(plan_id: str, spec: LoadSheetCreate,
                      version: int | None = None):
    snapshot, ver = _plan_snapshot(plan_id, version)
    sheet_id = storage.new_load_sheet_id()
    now = _now()
    doc = _build_doc(sheet_id, spec.name, plan_id, ver, snapshot, spec,
                     "draft", now, None)
    storage.insert_load_sheet(sheet_id, plan_id, ver, doc)
    return _json_text({"load_sheet": doc}, 201)


@router.get("/load-sheets")
def list_load_sheets(plan_id: str | None = None):
    return {"load_sheets": storage.list_load_sheets(plan_id)}


@router.get("/load-sheets/{sheet_id}")
def get_load_sheet(sheet_id: str):
    doc, _, _ = _load(sheet_id)
    return _json_text({"load_sheet": doc})


@router.post("/load-sheets/{sheet_id}/revisions", status_code=201)
def revise_load_sheet(sheet_id: str, rev: LoadSheetRevision):
    doc, plan_id, ver = _load(sheet_id)
    _require_draft(doc, "生成修订版（已采用校核单不可改）")
    snapshot, _ = _plan_snapshot(plan_id, ver)

    def take(field, default):
        v = getattr(rev, field)
        return v if v is not None else default

    # 以原文档重建创建参数：未提供的字段沿用原校核单
    old_case = doc["case"]
    old_src = doc["tension_source"]
    old_req = doc["pretension_search_request"]
    src = rev.tension_source
    if src is None:
        if old_src["type"] == "uniform":
            from .schemas import UniformTensionSource
            src = UniformTensionSource(left_n=old_src["left_n"],
                                       right_n=old_src["right_n"])
        else:
            from .schemas import BatchTensionSource
            src = BatchTensionSource(batch_id=old_src["batch_id"])
    from .schemas import (AngleSweep, PretensionSearchSpec,
                          ServiceLoads, TensionLimitOverride)
    sweep = rev.sweep or AngleSweep(
        start_deg=old_case["angles"]["start_deg"],
        end_deg=old_case["angles"]["end_deg"],
        step_deg=old_case["angles"]["step_deg"])
    old_l = old_case["loads"]
    loads = rev.loads or ServiceLoads(
        radial_n=old_l["radial_n"], lateral_n=old_l["lateral_n"],
        torque_nm=old_l["torque_nm"], torque_direction=old_l["torque_direction"],
        load_factor=old_l["load_factor"])
    ov = rev.tension_limits_override
    if ov is None and old_case["tension_limits_override"] is not None:
        # 未显式提供时沿用原校核单的左右侧窗口
        ov = TensionLimitOverride.model_validate(
            old_case["tension_limits_override"])
    search_req = rev.pretension_search
    if search_req is None and old_req is not None:
        search_req = PretensionSearchSpec(**old_req)
    merged = LoadSheetCreate(
        name=rev.name or doc["name"],
        tension_source=src,
        sweep=sweep,
        loads=loads,
        tension_min_n=take("tension_min_n", old_case["tension_min_n"]),
        tension_max_n=take("tension_max_n", old_case["tension_max_n"]),
        tension_limits_override=ov,
        residual_tolerance=take("residual_tolerance",
                                old_case["residual_tolerance"]),
        pretension_search=search_req,
    )
    new_id = storage.new_load_sheet_id()
    now = _now()
    new_doc = _build_doc(new_id, merged.name, plan_id, ver, snapshot, merged,
                         "draft", now, sheet_id)
    storage.insert_load_sheet(new_id, plan_id, ver, new_doc)
    return _json_text({"load_sheet": new_doc}, 201)


# ---------------------------------------------------------------------------
# 采用（冻结）与放弃
# ---------------------------------------------------------------------------

@router.post("/load-sheets/{sheet_id}/adopt")
def adopt_load_sheet(sheet_id: str, req: AdoptRequest | None = None):
    doc, plan_id, ver = _load(sheet_id)
    _require_draft(doc, "采用校核单")
    selected_index = None
    if req is not None:
        selected_index = req.selected_index
    # 采用时重新计算以核验一致性：相同输入必须得到相同结果
    old_result = doc["result"]
    if selected_index is not None:
        if "pretension_search" not in old_result:
            raise WheelError(
                "PRETENSION_OPTION_NOT_FOUND",
                "该校核单未进行预张力搜索，不能选定方案编号",
                {"selected_index": selected_index, "option_count": 0})
        count = len(old_result["pretension_search"]["options"])
        if selected_index >= count:
            raise WheelError(
                "PRETENSION_OPTION_NOT_FOUND",
                f"预张力方案编号 {selected_index} 不存在（共 {count} 个）",
                {"selected_index": selected_index, "option_count": count})
    doc["status"] = "adopted"
    now = _now()
    doc["updated_at"] = now
    if "pretension_search" in old_result:
        if selected_index is None:
            selected_index = old_result["pretension_search"]["selected_index"]
        old_result["pretension_search"]["selected_index"] = selected_index
        old_result["pretension_search"]["selected"] = \
            old_result["pretension_search"]["options"][selected_index]
        doc["adopted"] = {
            "adopted_at": now,
            "selected_index": selected_index,
            "selected_label":
                old_result["pretension_search"]["options"][selected_index]["label"],
            "adjustments":
                old_result["pretension_search"]["options"][selected_index]["adjustments"],
        }
    else:
        doc["adopted"] = {"adopted_at": now, "selected_index": None,
                          "selected_label": None, "adjustments": []}
    storage.save_load_sheet(sheet_id, doc, "adopted")
    return _json_text({"load_sheet": doc})


@router.delete("/load-sheets/{sheet_id}", status_code=204)
def discard_load_sheet(sheet_id: str):
    doc, _, _ = _load(sheet_id)
    _require_draft(doc, "放弃校核单")
    storage.delete_load_sheet(sheet_id)
    return Response(status_code=204)
