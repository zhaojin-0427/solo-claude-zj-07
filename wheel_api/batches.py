"""调校批次路由：从不可变方案版本取数，逐轮采集、预测、确认、定稿。

批次状态机：
    collecting（采集中） --measurements--> collecting（草稿，可重交）
    collecting --propose(可选)/locks--> collecting
    collecting --POST rounds/confirm--> adjusting（调整中，冻结该轮测量+动作+结果）
    adjusting --cancel--> collecting（放弃该轮，回到上轮已确认状态）
    collecting/adjusting --finalize--> finalized（已定稿，拒绝一切追加）

存储：batches 表保存可变当前状态；每次状态变更向 batch_events 只增追加
一条事件快照，完整调校轨迹（含采集中的草稿更替）均可重放。
"""

from __future__ import annotations

import json
import math
from datetime import datetime, timezone

from fastapi import APIRouter
from fastapi.responses import Response

from . import geometry, storage, tuning
from .errors import WheelError
from .schemas import (
    ConfirmRoundRequest,
    LockRequest,
    MeasurementSubmit,
    TuningBatchCreate,
)

router = APIRouter()


def _now() -> str:
    return datetime.now(timezone.utc).isoformat()


def _json_text(payload: dict, status_code: int = 200) -> Response:
    return Response(content=storage.canonical(payload),
                    media_type="application/json", status_code=status_code)


# ---------------------------------------------------------------------------
# 取数与状态装配
# ---------------------------------------------------------------------------

def _plan_snapshot(plan_id: str, version: int | None) -> tuple[dict, int]:
    row = storage.plan_row(plan_id)
    if row is None:
        raise WheelError("PLAN_NOT_FOUND", f"方案 {plan_id} 不存在",
                         {"plan_id": plan_id}, status=404)
    current_version = row[3]
    ver = current_version if version is None else version
    text = storage.get_snapshot(plan_id, ver)
    if text is None:
        raise WheelError(
            "VERSION_NOT_FOUND",
            f"方案 {plan_id} 的版本 {ver} 不存在",
            {"plan_id": plan_id, "version": ver}, status=404,
        )
    return json.loads(text), ver


def _require_status(state: dict, allowed: set[str], action: str):
    if state["status"] not in allowed:
        raise WheelError(
            "BATCH_STATUS_INVALID",
            f"批次当前状态为 {state['status']}，不能{action}（允许状态: {sorted(allowed)}）",
            {"status": state["status"], "allowed": sorted(allowed), "action": action},
        )


def _load(batch_id: str) -> tuple[dict, str, int]:
    row = storage.batch_row(batch_id)
    if row is None:
        raise WheelError("BATCH_NOT_FOUND", f"调校批次 {batch_id} 不存在",
                         {"batch_id": batch_id}, status=404)
    state = json.loads(row[3])
    # JSON 对象键为字符串：圈孔号、累计转动量键还原为整数
    state["locks"] = {int(k): v for k, v in state.get("locks", {}).items()}
    state["cumulative_turns"] = {
        int(k): v for k, v in state.get("cumulative_turns", {}).items()}
    return state, row[1], row[2]


def _holes(state: dict) -> set[int]:
    return {h["rim_hole"] for h in state["spokes"]}


def _pitch(state: dict) -> float:
    return 2.0 * math.pi / len(state["spokes"])


def _influence(state: dict) -> dict:
    spokes = _state_spokes(state)
    return tuning.build_influence(
        spokes, _pitch(state),
        state["rim_influence_radial_mm_per_turn"],
        state["rim_influence_lateral_mm_per_turn"],
        state["thread_pitch_mm"],
        state.get("tension_transfer", 1.0),
    )


def _state_spokes(state: dict) -> list[dict]:
    return [{**h, "angle_rad": math.radians(h["angle_deg"]),
             "spoke_diameter_mm": state["spoke_diameter_mm"]}
            for h in state["spokes"]]


def _initial_state(batch_id: str, name: str, plan_id: str, ver: int,
                   spec: TuningBatchCreate, snapshot: dict) -> dict:
    spokes_raw = tuning.extract_spokes(snapshot)
    angles = [s["angle_rad"] for s in spokes_raw]
    raw_weights = tuning._gap_weights(angles)
    # 舍入后重新归一化：保证合计严格为 1，加权碟形/偏心不因权重漂移产生 0.004 偏差
    rounded = [geometry.r3(w) for w in raw_weights]
    residual = geometry.r3(1.0 - sum(rounded))
    rounded[-1] = geometry.r3(rounded[-1] + residual)
    weights = rounded
    ov = spec.tension_limits_override
    return {
        "batch_id": batch_id,
        "name": name,
        "status": "collecting",
        "plan_ref": {
            "plan_id": plan_id,
            "version": ver,
            "formula_version": snapshot["formula_version"],
        },
        "calibration_formula_version": tuning.CALIBRATION_FORMULA_VERSION,
        "tuning_formula_version": tuning.TUNING_FORMULA_VERSION,
        "tuning_formulas": tuning.TUNING_FORMULAS,
        "calibration_curve": [
            {"reading": p.reading, "tension_n": p.tension_n}
            for p in sorted(spec.calibration_curve, key=lambda p: p.reading)
        ],
        "radial_zero_mm": spec.radial_zero_mm,
        "lateral_zero_mm": spec.lateral_zero_mm,
        "thread_pitch_mm": spec.thread_pitch_mm,
        "rim_influence_radial_mm_per_turn": spec.rim_influence_radial_mm_per_turn,
        "rim_influence_lateral_mm_per_turn": spec.rim_influence_lateral_mm_per_turn,
        "tension_transfer": spec.tension_transfer,
        "tension_min_n": spec.tension_min_n,
        "tension_max_n": spec.tension_max_n,
        "tension_limits_override": (
            {
                "left": ov.left.model_dump() if ov and ov.left else None,
                "right": ov.right.model_dump() if ov and ov.right else None,
            }
            if ov is not None else None
        ),
        "radial_tolerance_mm": spec.radial_tolerance_mm,
        "lateral_tolerance_mm": spec.lateral_tolerance_mm,
        "max_turns_per_spoke": spec.max_turns_per_spoke,
        "step_start_angle_deg": spec.step_start_angle_deg,
        "spoke_diameter_mm": snapshot["geometry"]["spoke_diameter_mm"],
        "spokes": [
            {"rim_hole": s["rim_hole"], "side": s["side"],
             "angle_deg": geometry.r3(math.degrees(s["angle_rad"])),
             "length_mm": s["length_mm"]}
            for s in spokes_raw
        ],
        "gap_weights": weights,
        "locks": {},
        "draft": None,
        "rounds": [],
        "cumulative_turns": {},
        "finalized_at": None,
    }


def _public_view(state: dict) -> dict:
    """对外视图：完整状态（测量点、各轮轨迹、锁定、累计转动）。"""
    return state


# ---------------------------------------------------------------------------
# 创建 / 列出 / 读取
# ---------------------------------------------------------------------------

@router.post("/plans/{plan_id}/batches", status_code=201)
def create_batch(plan_id: str, spec: TuningBatchCreate, version: int | None = None):
    snapshot, ver = _plan_snapshot(plan_id, version)
    batch_id = storage.new_batch_id()
    state = _initial_state(batch_id, spec.name, plan_id, ver, spec, snapshot)
    event = {"batch_id": batch_id, "kind": "batch_created",
             "plan_ref": state["plan_ref"], "state": state}
    storage.insert_batch(batch_id, plan_id, ver, state, "batch_created", event)
    return _json_text({"batch": _public_view(state)}, 201)


@router.get("/batches")
def list_all_batches(plan_id: str | None = None):
    return {"batches": storage.list_batches(plan_id)}


@router.get("/batches/{batch_id}")
def get_batch(batch_id: str):
    state, _, _ = _load(batch_id)
    return _json_text({"batch": _public_view(state)})


# ---------------------------------------------------------------------------
# 锁定 / 解锁（仅采集中）
# ---------------------------------------------------------------------------

@router.post("/batches/{batch_id}/locks")
def update_locks(batch_id: str, req: LockRequest):
    state, _, _ = _load(batch_id)
    _require_status(state, {"collecting"}, "锁定或解锁辐条")
    holes = _holes(state)
    bad = sorted((set(req.lock) | set(req.unlock)) - holes)
    if bad:
        raise WheelError(
            "BATCH_HOLE_UNKNOWN",
            f"孔号不属于来源方案: {bad}",
            {"rim_holes": bad, "valid_range_note": "以来源方案圈孔编号为准"},
        )
    locked, unlocked = [], []
    for h in req.unlock:
        if h in state["locks"]:
            state["locks"].pop(h)
            unlocked.append(h)
    for h in req.lock:
        if h not in state["locks"]:
            side = next(s["side"] for s in state["spokes"] if s["rim_hole"] == h)
            state["locks"][h] = {"rim_hole": h, "side": side, "locked_at": _now()}
            locked.append(h)
    event = {"batch_id": batch_id, "kind": "locks_updated",
             "locked": locked, "unlocked": unlocked, "state": state}
    storage.update_batch(batch_id, state, "locks_updated", event)
    return _json_text({"batch": _public_view(state),
                       "locked": sorted(locked), "unlocked": sorted(unlocked)})


# ---------------------------------------------------------------------------
# 测量提交（采集中草稿；可覆盖重交；错误一律指到测点）
# ---------------------------------------------------------------------------

@router.post("/batches/{batch_id}/measurements")
def submit_measurement(batch_id: str, payload: MeasurementSubmit):
    state, _, _ = _load(batch_id)
    spokes = _state_spokes(state)
    curve = [(p["reading"], p["tension_n"]) for p in state["calibration_curve"]]
    readings = [r.model_dump() for r in payload.readings]
    # 测点校验先于状态机：即使调整中/已定稿，缺测、重复、越界等错误
    # 仍能指到本次提交的测点
    by_idx = tuning.validate_readings(readings, spokes, curve)
    if state["status"] != "collecting":
        raise WheelError(
            "BATCH_STATUS_INVALID",
            f"批次当前状态为 {state['status']}，不能追加测量数据"
            f"（仅采集中允许提交；已定稿批次不可追加）",
            {
                "status": state["status"],
                "allowed": ["collecting"],
                "action": "submit_measurements",
                "submitted_count": len(readings),
                "submitted_rim_holes": [r["rim_hole"] for r in readings],
            },
        )
    weights = state["gap_weights"]
    analysis = tuning.analyze_measurement(state, spokes, weights, by_idx)

    replaced = state["draft"] is not None
    state["draft"] = {"submitted_at": _now(), "analysis": analysis}
    event = {"batch_id": batch_id,
             "kind": "measurement_replaced" if replaced else "measurement_submitted",
             "replaced": replaced, "analysis": analysis, "state": state}
    storage.update_batch(batch_id, state, event["kind"], event)
    return _json_text({"batch": _public_view(state),
                       "draft_replaced": replaced,
                       "analysis": analysis})


# ---------------------------------------------------------------------------
# 方案预测（必须有本轮草稿；不产生状态变更）
# ---------------------------------------------------------------------------

def _proposals_for(state: dict) -> list[dict]:
    """根据当前草稿生成候选（确定性：相同状态逐字节一致）。"""
    spokes = _state_spokes(state)
    inf = _influence(state)
    return tuning.build_candidates(
        state, spokes, state["gap_weights"], inf,
        state["draft"]["analysis"],
        math.radians(state["step_start_angle_deg"]),
    )


@router.get("/batches/{batch_id}/proposals")
def propose(batch_id: str):
    state, _, _ = _load(batch_id)
    _require_status(state, {"collecting", "adjusting"}, "请求调校方案")
    if state["status"] == "adjusting":
        # 调整中返回冻结轮已确认的候选集，保证重复读取一致
        round_ = state["rounds"][-1]
        return _json_text({"round": round_["round"], "candidates": round_["candidates"]})
    if state["draft"] is None:
        raise WheelError(
            "BATCH_NO_MEASUREMENT",
            "本轮尚未采集测量数据：先 POST /batches/{id}/measurements",
            {"batch_id": batch_id},
        )
    cached = state["draft"].get("candidates")
    if cached is None:
        cached = _proposals_for(state)
        # 缓存进草稿：确认时读取同一份，避免"看到"与"确认"的方案漂移
        state["draft"]["candidates"] = cached
        event = {"batch_id": batch_id, "kind": "proposals_cached", "state": state}
        storage.update_batch(batch_id, state, "proposals_cached", event)
    return _json_text({
        "round": len(state["rounds"]) + 1,
        "status": "collecting",
        "formula_version": state["tuning_formula_version"],
        "candidates": cached,
    })


# ---------------------------------------------------------------------------
# 确认一轮：冻结测量 + 动作 + 结果，进入调整中
# ---------------------------------------------------------------------------

@router.post("/batches/{batch_id}/rounds/confirm")
def confirm_round(batch_id: str, req: ConfirmRoundRequest):
    state, _, _ = _load(batch_id)
    _require_status(state, {"collecting"}, "确认一轮")
    if state["draft"] is None:
        raise WheelError(
            "BATCH_NO_MEASUREMENT",
            "本轮尚未采集测量数据：先 POST /batches/{id}/measurements",
            {"batch_id": batch_id},
        )
    candidates = state["draft"].get("candidates")
    if candidates is None:
        candidates = _proposals_for(state)
    if req.candidate_index >= len(candidates):
        raise WheelError(
            "CANDIDATE_NOT_FOUND",
            f"候选编号 {req.candidate_index} 不存在（本轮共 {len(candidates)} 个候选 0..{len(candidates) - 1}）",
            {"candidate_index": req.candidate_index, "candidate_count": len(candidates)},
        )
    chosen = candidates[req.candidate_index]
    round_no = len(state["rounds"]) + 1

    # 累计转动量（供下一轮起点与定稿轨迹）
    for a in chosen["spoke_actions"]:
        key = str(a["rim_hole"])
        signed = a["turns"] if a["direction"] == "tighten" else -a["turns"]
        state["cumulative_turns"][key] = geometry.r3(
            state["cumulative_turns"].get(key, 0.0) + signed)

    frozen = {
        "round": round_no,
        "confirmed_at": _now(),
        "measurement": state["draft"]["analysis"],
        "selected_candidate_index": req.candidate_index,
        "candidate": chosen,
        "candidates": candidates,
        "locks_at_confirm": {h: v["side"] for h, v in state["locks"].items()},
        "cumulative_turns": dict(sorted(state["cumulative_turns"].items(),
                                        key=lambda kv: int(kv[0]))),
    }
    state["rounds"].append(frozen)
    state["draft"] = None
    state["status"] = "adjusting"
    event = {"batch_id": batch_id, "kind": "round_confirmed", "round": frozen}
    storage.update_batch(batch_id, state, "round_confirmed", event)
    return _json_text({"batch": _public_view(state), "round": frozen}, 201)


# ---------------------------------------------------------------------------
# 取消调整：回到采集中，丢弃未完成轮（已确认轮不可回退）
# ---------------------------------------------------------------------------

@router.post("/batches/{batch_id}/rounds/cancel")
def cancel_round(batch_id: str):
    state, _, _ = _load(batch_id)
    _require_status(state, {"adjusting"}, "取消当前调整轮")
    round_no = len(state["rounds"])
    state["status"] = "collecting"
    event = {"batch_id": batch_id, "kind": "adjustment_cancelled",
             "round": round_no, "note": "该轮已冻结快照保留在轨迹中；状态回到采集中以开始下一轮"}
    storage.update_batch(batch_id, state, "adjustment_cancelled", event)
    return _json_text({"batch": _public_view(state)})


# ---------------------------------------------------------------------------
# 完成一轮并进入下一轮：调整中 -> 采集中（快照之后继续）
# ---------------------------------------------------------------------------

@router.post("/batches/{batch_id}/rounds/complete")
def complete_round(batch_id: str):
    state, _, _ = _load(batch_id)
    _require_status(state, {"adjusting"}, "完成本轮调整")
    round_no = len(state["rounds"])
    state["status"] = "collecting"
    event = {"batch_id": batch_id, "kind": "round_completed",
             "round": round_no,
             "note": "测量、动作与结果已冻结；下一轮从该快照继续"}
    storage.update_batch(batch_id, state, "round_completed", event)
    return _json_text({"batch": _public_view(state),
                       "next_round": round_no + 1})


# ---------------------------------------------------------------------------
# 定稿：保留来源方案版本与完整调校轨迹；之后拒绝一切追加
# ---------------------------------------------------------------------------

@router.post("/batches/{batch_id}/finalize")
def finalize_batch(batch_id: str):
    state, _, _ = _load(batch_id)
    _require_status(state, {"collecting", "adjusting"}, "定稿批次")
    if not state["rounds"]:
        raise WheelError(
            "BATCH_FINALIZE_EMPTY",
            "尚无任何已确认调校轮，不能定稿（至少完成一轮采集与确认）",
            {"batch_id": batch_id},
        )
    state["status"] = "finalized"
    state["finalized_at"] = _now()
    event = {"batch_id": batch_id, "kind": "finalized",
             "finalized_at": state["finalized_at"],
             "plan_ref": state["plan_ref"],
             "round_count": len(state["rounds"]),
             "trajectory": _trajectory(state)}
    storage.update_batch(batch_id, state, "finalized", event)
    return _json_text({"batch": _public_view(state),
                       "finalized_at": state["finalized_at"]})


def _trajectory(state: dict) -> dict:
    return {
        "plan_ref": state["plan_ref"],
        "tuning_formula_version": state["tuning_formula_version"],
        "calibration_formula_version": state["calibration_formula_version"],
        "rounds": [
            {
                "round": r["round"],
                "confirmed_at": r["confirmed_at"],
                "selected_candidate_index": r["selected_candidate_index"],
                "measurement_metrics": r["measurement"]["metrics"],
                "result_metrics": r["candidate"]["metrics"],
                "spoke_actions": r["candidate"]["spoke_actions"],
                "steps": [
                    {"step": s["step"], "rim_hole": s["rim_hole"], "side": s["side"],
                     "direction": s["direction"], "turns": s["turns"],
                     "predicted": s["predicted"]}
                    for s in r["candidate"]["steps"]
                ],
                "cumulative_turns": r["cumulative_turns"],
            }
            for r in state["rounds"]
        ],
    }


@router.get("/batches/{batch_id}/trajectory")
def get_trajectory(batch_id: str):
    state, _, _ = _load(batch_id)
    events = storage.list_batch_events(batch_id)
    return _json_text({
        "batch_id": batch_id,
        "status": state["status"],
        "plan_ref": state["plan_ref"],
        "formula_versions": {
            "plan": state["plan_ref"]["formula_version"],
            "tuning": state["tuning_formula_version"],
            "calibration": state["calibration_formula_version"],
        },
        "trajectory": _trajectory(state),
        "events": [
            {"seq": e["seq"], "kind": e["kind"], "created_at": e["created_at"]}
            for e in events
        ],
    })
