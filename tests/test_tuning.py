"""调校批次端到端测试：取数、采集校验、指标换算、方案预测、确认冻结、定稿。"""

import math
import os

_TMP_DB = "/tmp/test_tuning.db"
if os.path.exists(_TMP_DB):
    os.remove(_TMP_DB)
os.environ["WHEEL_API_DB"] = _TMP_DB

import pytest
from fastapi.testclient import TestClient

from wheel_api.app import app
from wheel_api import tuning

client = TestClient(app)

SPEC = {
    "name": "tune-wheel",
    "rim": {"erd_mm": 600.0, "holes": 36, "valve_position": 35},
    "hub": {
        "holes_per_flange": 18,
        "flange_pcd_left_mm": 58.0,
        "flange_pcd_right_mm": 45.0,
        "center_to_flange_left_mm": 35.0,
        "center_to_flange_right_mm": 20.0,
    },
    "left": {"cross": 3},
    "right": {"cross": 3},
}

BATCH = {
    "name": "tune-1",
    "calibration_curve": [
        {"reading": 10.0, "tension_n": 400.0},
        {"reading": 20.0, "tension_n": 900.0},
        {"reading": 30.0, "tension_n": 1500.0},
    ],
    "radial_zero_mm": 0.0,
    "lateral_zero_mm": 0.0,
    "thread_pitch_mm": 0.454,
    "rim_influence_radial_mm_per_turn": 0.08,
    "rim_influence_lateral_mm_per_turn": 0.20,
    "tension_transfer": 0.5,
    "tension_min_n": 400.0,
    "tension_max_n": 1400.0,
    "radial_tolerance_mm": 0.3,
    "lateral_tolerance_mm": 0.3,
}


@pytest.fixture(scope="module")
def plan_id():
    r = client.post("/plans", json=SPEC)
    assert r.status_code == 201, r.text
    return r.json()["plan_id"]


@pytest.fixture()
def batch(plan_id):
    r = client.post(f"/plans/{plan_id}/batches?version=1", json=BATCH)
    assert r.status_code == 201, r.text
    return r.json()["batch"]


def _holes(batch_id):
    b = client.get(f"/batches/{batch_id}").json()["batch"]
    return [s["rim_hole"] for s in b["spokes"]]


def _readings(holes, gauge=20.0, radial=0.0, lateral=0.0):
    return [
        {"rim_hole": h, "gauge_reading": gauge,
         "radial_mm": radial(i) if callable(radial) else radial,
         "lateral_mm": lateral(i) if callable(lateral) else lateral}
        for i, h in enumerate(holes)
    ]


# ---------------------------------------------------------------------------
# 创建与取数
# ---------------------------------------------------------------------------

def test_create_batch_pins_plan_version(plan_id, batch):
    assert batch["status"] == "collecting"
    assert batch["plan_ref"] == {"plan_id": plan_id, "version": 1,
                                 "formula_version": "wheel-geometry/1.2"}
    assert len(batch["spokes"]) == 36
    # 实际孔位角度随快照取来，非等距权重在等距轮上恒为 1/N
    assert sum(batch["gap_weights"]) == pytest.approx(1.0, abs=1e-6)
    assert batch["tuning_formula_version"] == "wheel-tuning/1.0"
    assert batch["calibration_curve"][1] == {"reading": 20.0, "tension_n": 900.0}
    # 方案后续再出新版本，批次仍引用版本 1
    client.post(f"/plans/{plan_id}/versions", json={**SPEC, "name": "v2"})
    assert client.get(f"/batches/{batch['batch_id']}").json()["batch"]["plan_ref"]["version"] == 1


def test_create_batch_404():
    r = client.post("/plans/whl_missing/batches", json=BATCH)
    assert r.status_code == 404
    assert r.json()["error"]["code"] == "PLAN_NOT_FOUND"


def test_calibration_curve_validation(plan_id):
    bad = {**BATCH, "calibration_curve": [
        {"reading": 20, "tension_n": 900}, {"reading": 20, "tension_n": 950}]}
    r = client.post(f"/plans/{plan_id}/batches", json=bad)
    assert r.status_code == 422 and "严格递增" in r.text
    bad = {**BATCH, "calibration_curve": [
        {"reading": 10, "tension_n": 900}, {"reading": 20, "tension_n": 800}]}
    r = client.post(f"/plans/{plan_id}/batches", json=bad)
    assert r.status_code == 422 and "单调不减" in r.text


def test_max_turns_must_be_eighth_multiple(plan_id):
    r = client.post(f"/plans/{plan_id}/batches", json={**BATCH, "max_turns_per_spoke": 0.3})
    assert r.status_code == 422 and "1/8" in r.text


# ---------------------------------------------------------------------------
# 采集：换算与测点错误
# ---------------------------------------------------------------------------

def test_measurement_tension_conversion_and_metrics(plan_id, batch):
    bid = batch["batch_id"]
    holes = _holes(bid)
    # 读数 10/20/30 -> 400/900/1500；读数 25 -> 1200（线性插值）
    readings = _readings(holes, lateral=0.5)
    readings[5]["gauge_reading"] = 25.0
    r = client.post(f"/batches/{bid}/measurements", json={"readings": readings})
    assert r.status_code == 200, r.text
    a = r.json()["analysis"]
    by_hole = {p["rim_hole"]: p for p in a["points"]}
    assert by_hole[holes[0]]["tension_n"] == 900.0
    assert by_hole[holes[5]]["tension_n"] == 1200.0
    # 全部横向 0.5（正=偏右）：碟形偏移 0.5，径向无跳动
    assert a["metrics"]["dish_offset_mm"] == pytest.approx(0.5, abs=1e-6)
    assert a["metrics"]["eccentricity_mm"] == 0.0
    assert a["metrics"]["radial_peak_mm"] == 0.0
    # 离散度：等张力 900 为 0；单点 1200 的同侧 5 孔窗标准差 = 120（=300/2.5）
    # 在若干个窗上均值为 33.333
    assert a["metrics"]["local_tension_dispersion_n"] == pytest.approx(33.333, abs=0.01)
    assert a["tension_by_side"]["right"]["mean_n"] == 900.0
    # 零位扣除：读数 0.3、零位 0.3 -> 相对跳动 0（另建批次）
    b2 = client.post(f"/plans/{plan_id}/batches",
                     json={**BATCH, "radial_zero_mm": 0.3, "lateral_zero_mm": 0.3})
    bid2 = b2.json()["batch"]["batch_id"]
    r = client.post(f"/batches/{bid2}/measurements",
                    json={"readings": _readings(_holes(bid2), radial=0.3, lateral=0.3)})
    m = r.json()["analysis"]["metrics"]
    assert m["radial_peak_mm"] == 0.0 and m["lateral_peak_mm"] == 0.0


def test_measurement_eccentricity_harmonic(plan_id, batch):
    bid = batch["batch_id"]
    holes = _holes(bid)
    readings = _readings(holes, radial=lambda i: math.cos(math.radians(i * 10)))
    r = client.post(f"/batches/{bid}/measurements", json={"readings": readings})
    m = r.json()["analysis"]["metrics"]
    # r_i = cos(theta_i) 的一阶谐波分量恰为 0.5（cos² 均值）
    assert m["eccentricity_mm"] == pytest.approx(0.5, abs=0.02)
    assert m["eccentricity_angle_deg"] == pytest.approx(0.0, abs=2.0)


def test_measurement_accepts_cyclic_rotation(plan_id, batch):
    bid = batch["batch_id"]
    holes = _holes(bid)
    rd = _readings(holes)
    r = client.post(f"/batches/{bid}/measurements", json={"readings": rd[7:] + rd[:7]})
    assert r.status_code == 200, r.text  # 允许任意孔起步的循环移位


def test_measurement_order_error_points_to_hole(plan_id, batch):
    bid = batch["batch_id"]
    holes = _holes(bid)
    rd = _readings(holes)
    rd[3], rd[4] = rd[4], rd[3]
    r = client.post(f"/batches/{bid}/measurements", json={"readings": rd})
    assert r.status_code == 400
    err = r.json()["error"]
    assert err["code"] == "MEASUREMENT_ORDER_INVALID"
    assert err["details"]["index"] == 3
    assert "rim_hole" in err["details"]


def test_measurement_missing_hole(plan_id, batch):
    bid = batch["batch_id"]
    holes = _holes(bid)
    r = client.post(f"/batches/{bid}/measurements", json={"readings": _readings(holes)[:-1]})
    assert r.status_code == 400
    err = r.json()["error"]
    assert err["code"] == "MEASUREMENT_MISSING_HOLE"
    assert err["details"]["missing_rim_holes"] == [holes[-1]]


def test_measurement_duplicate_hole(plan_id, batch):
    bid = batch["batch_id"]
    rd = _readings(_holes(bid))
    r = client.post(f"/batches/{bid}/measurements", json={"readings": rd + [rd[0]]})
    assert r.status_code == 400
    assert r.json()["error"]["code"] == "MEASUREMENT_DUPLICATE_HOLE"


def test_measurement_out_of_calibration_range(plan_id, batch):
    bid = batch["batch_id"]
    holes = _holes(bid)
    rd = _readings(holes)
    rd[2]["gauge_reading"] = 99.0
    r = client.post(f"/batches/{bid}/measurements", json={"readings": rd})
    assert r.status_code == 400
    err = r.json()["error"]
    assert err["code"] == "MEASUREMENT_OUT_OF_RANGE"
    assert err["details"]["points"][0]["rim_hole"] == holes[2]
    assert err["details"]["range"] == [10.0, 30.0]


def test_measurement_unknown_hole(plan_id, batch):
    bid = batch["batch_id"]
    rd = _readings(_holes(bid))
    rd[0]["rim_hole"] = 999
    r = client.post(f"/batches/{bid}/measurements", json={"readings": rd})
    assert r.status_code == 400
    assert r.json()["error"]["code"] == "MEASUREMENT_HOLE_UNKNOWN"


def test_proposal_requires_measurement(plan_id, batch):
    bid = batch["batch_id"]
    r = client.get(f"/batches/{bid}/proposals")
    assert r.status_code == 400
    assert r.json()["error"]["code"] == "BATCH_NO_MEASUREMENT"


# ---------------------------------------------------------------------------
# 方案预测
# ---------------------------------------------------------------------------

def _proposals(bid, readings):
    assert client.post(f"/batches/{bid}/measurements",
                       json={"readings": readings}).status_code == 200
    r = client.get(f"/batches/{bid}/proposals")
    assert r.status_code == 200, r.text
    return r.json()["candidates"]


def test_proposals_baseline_and_ranking(plan_id, batch):
    bid = batch["batch_id"]
    holes = _holes(bid)
    cands = _proposals(bid, _readings(holes, lateral=0.5))
    assert cands[0]["label"] == "no_action"
    assert cands[0]["candidate_index"] == 0
    assert cands[0]["steps"] == [] and cands[0]["spoke_actions"] == []
    # 至少一个动作候选严格优于基线，且候选按官方字典序排序
    assert len(cands) >= 2
    keys = [(c["metrics"]["violation_count"], c["metrics"]["max_runout_mm"],
             c["metrics"]["local_tension_dispersion_n"], c["metrics"]["total_turns"])
            for c in cands]
    assert keys[1:] == sorted(keys[1:])
    assert keys[1] < keys[0]
    # 正横向偏移（轮圈偏右）的物理纠正：左侧收紧、右侧拧松
    # （左张角小、机械优势低；碟形从 0.5 压到接近 0 证实方向有效）
    best = cands[1]
    right_dirs = [a["direction"] for a in best["spoke_actions"] if a["side"] == "right"]
    left_dirs = [a["direction"] for a in best["spoke_actions"] if a["side"] == "left"]
    if right_dirs:
        assert right_dirs.count("loosen") >= right_dirs.count("tighten")
    if left_dirs:
        assert left_dirs.count("tighten") >= left_dirs.count("loosen")
    assert best["metrics"]["dish_offset_mm"] < 0.35
    for s in best["steps"]:
        assert s["turns"] in (0.125, 0.25, 0.5)
        assert "predicted" in s and "max_runout_mm" in s["predicted"]
    # 逐步预测的终态 == 汇总终态
    assert best["steps"][-1]["predicted"]["max_runout_mm"] == best["metrics"]["max_runout_mm"]


def test_proposal_uses_actual_hole_angles(plan_id, batch):
    # 与孔表方案联动：非等距孔位批次也能生成候选（实际角度建窗）
    spec_ht = {
        "name": "paired",
        "rim": {
            "erd_mm": 600.0, "holes": 12,
            "hole_table": [
                {"id": i, "angle_deg": a, "side": ("right" if i % 2 == 0 else "left")}
                for i, a in enumerate([0, 12, 60, 72, 120, 132, 180, 192, 240, 252, 300, 312])
            ],
        },
        "hub": {"holes_per_flange": 6, "flange_pcd_left_mm": 50.0,
                "flange_pcd_right_mm": 50.0, "center_to_flange_left_mm": 30.0,
                "center_to_flange_right_mm": 30.0},
        "left": {"cross": 0}, "right": {"cross": 0},
    }
    pid = client.post("/plans", json=spec_ht).json()["plan_id"]
    bid = client.post(f"/plans/{pid}/batches", json=BATCH).json()["batch"]["batch_id"]
    holes = _holes(bid)
    cands = _proposals(bid, _readings(holes, lateral=0.5))
    assert cands[0]["label"] == "no_action"
    assert len(cands) >= 2


def test_locked_spokes_not_adjusted(plan_id, batch):
    bid = batch["batch_id"]
    holes = _holes(bid)
    target = holes[10]
    r = client.post(f"/batches/{bid}/locks", json={"lock": [target]})
    assert r.status_code == 200, r.text
    assert int(list(r.json()["batch"]["locks"].keys())[0]) == target
    cands = _proposals(bid, _readings(holes, radial=lambda i: 0.5 if i == 10 else 0.0,
                                      lateral=0.5))
    for cand in cands[1:]:
        assert target not in [a["rim_hole"] for a in cand["spoke_actions"]]
        assert any(b["rim_hole"] == target and "locked" in b["reasons"]
                   for b in cand["blocked_spokes"])
    # 解锁后恢复可调
    client.post(f"/batches/{bid}/locks", json={"unlock": [target]})
    assert client.get(f"/batches/{bid}").json()["batch"]["locks"] == {}


def test_lock_unknown_hole(plan_id, batch):
    bid = batch["batch_id"]
    r = client.post(f"/batches/{bid}/locks", json={"lock": [400]})
    assert r.status_code == 400 and r.json()["error"]["code"] == "BATCH_HOLE_UNKNOWN"


def test_upper_limit_blocks_tightening(plan_id):
    # 初始张力 900，上限设 900：所有孔达到上限，收紧类动作全部禁止
    # （偏右 0.5 的碟形理论上需要右侧收紧，此时没有可行改善，只剩基线）
    bid = client.post(f"/plans/{plan_id}/batches",
                      json={**BATCH, "tension_max_n": 900.0}).json()["batch"]["batch_id"]
    holes = _holes(bid)
    cands = _proposals(bid, _readings(holes, lateral=0.5))
    # 基线上全部 36 根都带 at_upper_limit_no_tighten 标记
    blocked = {b["rim_hole"] for b in cands[0]["blocked_spokes"]
               if "at_upper_limit_no_tighten" in b["reasons"]}
    assert blocked == set(holes)
    # 任何动作候选中都不得出现越上限的收紧
    for cand in cands[1:]:
        for a in cand["spoke_actions"]:
            if a["direction"] == "tighten":
                assert a["tension_after_n"] <= 900.0 + 1e-6


# ---------------------------------------------------------------------------
# 状态机：确认冻结、调整中、完成、定稿
# ---------------------------------------------------------------------------

def _confirm_first_round(bid, readings, candidate_index=1):
    cands = _proposals(bid, readings)
    idx = min(candidate_index, len(cands) - 1)
    r = client.post(f"/batches/{bid}/rounds/confirm", json={"candidate_index": idx})
    assert r.status_code == 201, r.text
    return r.json(), cands[idx]


def test_confirm_freezes_measurement_actions_result(plan_id, batch):
    bid = batch["batch_id"]
    holes = _holes(bid)
    body, chosen = _confirm_first_round(bid, _readings(holes, lateral=0.5))
    assert body["batch"]["status"] == "adjusting"
    fr = body["round"]
    assert fr["round"] == 1
    # 冻结的测量、候选结果与确认时一致
    assert fr["candidate"]["metrics"] == chosen["metrics"]
    assert fr["measurement"]["metrics"]["dish_offset_mm"] == pytest.approx(0.5, abs=1e-6)
    # 调整中 proposals 返回冻结候选，重复读取逐字节一致
    a = client.get(f"/batches/{bid}/proposals")
    b = client.get(f"/batches/{bid}/proposals")
    assert a.status_code == 200 and a.content == b.content
    assert a.json()["candidates"][fr["selected_candidate_index"]]["metrics"] == chosen["metrics"]
    # 调整中禁止采集/锁定/再次确认
    assert client.post(f"/batches/{bid}/measurements",
                       json={"readings": _readings(holes)}).status_code == 400
    assert client.post(f"/batches/{bid}/locks", json={"lock": [0]}).status_code == 400
    assert client.post(f"/batches/{bid}/rounds/confirm",
                       json={"candidate_index": 0}).status_code == 400


def test_next_round_continues_from_snapshot(plan_id, batch):
    bid = batch["batch_id"]
    holes = _holes(bid)
    body, chosen = _confirm_first_round(bid, _readings(holes, lateral=0.5))
    cum = body["round"]["cumulative_turns"]
    assert cum, "确认动作后应有累计转动量"
    r = client.post(f"/batches/{bid}/rounds/complete")
    assert r.status_code == 200 and r.json()["next_round"] == 2
    assert r.json()["batch"]["status"] == "collecting"
    # 第二轮从快照继续：累计转动量保留，轮 1 不可变
    post_t = {a["rim_hole"]: a["tension_after_n"] for a in chosen["spoke_actions"]}

    def g(t):
        return 10.0 if t <= 400 else (30.0 if t >= 1500 else 10 + 20 * (t - 400) / 1100)

    rd2 = [{"rim_hole": h, "gauge_reading": g(post_t.get(h, 900.0)),
            "radial_mm": 0.0, "lateral_mm": chosen["metrics"]["dish_offset_mm"]}
           for h in holes]
    assert client.post(f"/batches/{bid}/measurements", json={"readings": rd2}).status_code == 200
    body2, _ = _confirm_first_round(bid, rd2, candidate_index=0)
    assert body2["round"]["round"] == 2
    assert client.get(f"/batches/{bid}").json()["batch"]["rounds"][0]["candidate"]["metrics"] \
        == chosen["metrics"]


def test_confirm_bad_candidate_index(plan_id, batch):
    bid = batch["batch_id"]
    holes = _holes(bid)
    _proposals(bid, _readings(holes, lateral=0.5))
    r = client.post(f"/batches/{bid}/rounds/confirm", json={"candidate_index": 99})
    assert r.status_code == 400 and r.json()["error"]["code"] == "CANDIDATE_NOT_FOUND"


def test_cancel_round(plan_id, batch):
    bid = batch["batch_id"]
    holes = _holes(bid)
    _confirm_first_round(bid, _readings(holes, lateral=0.5), candidate_index=0)
    r = client.post(f"/batches/{bid}/rounds/cancel")
    assert r.status_code == 200 and r.json()["batch"]["status"] == "collecting"
    # 已冻结轮保留在轨迹中
    assert len(r.json()["batch"]["rounds"]) == 1
    # 采集后草案为空（确认时已清空），需要重新测量
    assert client.get(f"/batches/{bid}/proposals").status_code == 400


def test_finalize_requires_round(plan_id, batch):
    bid = batch["batch_id"]
    r = client.post(f"/batches/{bid}/finalize")
    assert r.status_code == 400
    assert r.json()["error"]["code"] == "BATCH_FINALIZE_EMPTY"


def test_finalize_blocks_appends_and_keeps_trajectory(plan_id, batch):
    bid = batch["batch_id"]
    holes = _holes(bid)
    _confirm_first_round(bid, _readings(holes, lateral=0.5), candidate_index=0)
    client.post(f"/batches/{bid}/rounds/complete")
    r = client.post(f"/batches/{bid}/finalize")
    assert r.status_code == 200 and r.json()["batch"]["status"] == "finalized"
    assert client.post(f"/batches/{bid}/measurements",
                       json={"readings": _readings(holes)}).status_code == 400
    assert client.post(f"/batches/{bid}/locks", json={"lock": [0]}).status_code == 400
    assert client.post(f"/batches/{bid}/rounds/confirm",
                       json={"candidate_index": 0}).status_code == 400
    assert client.post(f"/batches/{bid}/finalize").status_code == 400
    tr = client.get(f"/batches/{bid}/trajectory").json()
    assert tr["status"] == "finalized"
    assert tr["plan_ref"]["plan_id"] == plan_id and tr["plan_ref"]["version"] == 1
    assert len(tr["trajectory"]["rounds"]) == 1
    round1 = tr["trajectory"]["rounds"][0]
    assert round1["measurement_metrics"]["dish_offset_mm"] == pytest.approx(0.5, abs=1e-6)
    assert round1["steps"] == []  # 选了 no_action
    kinds = [e["kind"] for e in tr["events"]]
    assert kinds[0] == "batch_created" and kinds[-1] == "finalized"
    assert "round_confirmed" in kinds


def test_list_batches_and_404(plan_id, batch):
    assert client.get("/batches/trn_nonexistent").status_code == 404
    rows = client.get("/batches", params={"plan_id": plan_id}).json()["batches"]
    assert any(b["batch_id"] == batch["batch_id"] for b in rows)
    assert rows[0]["status"] in ("collecting", "adjusting", "finalized")


# ---------------------------------------------------------------------------
# 领域单元
# ---------------------------------------------------------------------------

def test_gap_weights_uniform():
    angles = [2 * math.pi * i / 8 for i in range(8)]
    w = tuning._gap_weights(angles)
    assert w == pytest.approx([1 / 8] * 8, abs=1e-9)


def test_per_side_tension_override(plan_id, batch):
    # 左侧上限 800、初始 900：左孔全部标记不可收紧；右侧仍按全局 1400
    body = {**BATCH, "tension_limits_override": {"left": {"min_n": 400, "max_n": 800}}}
    bid = client.post(f"/plans/{plan_id}/batches?version=1", json=body).json()["batch"]["batch_id"]
    holes = _holes(bid)
    spokes = {s["rim_hole"]: s["side"]
              for s in client.get(f"/batches/{bid}").json()["batch"]["spokes"]}
    cands = _proposals(bid, _readings(holes, lateral=0.5))
    left_flagged = {b["rim_hole"] for b in cands[0]["blocked_spokes"]
                    if spokes[b["rim_hole"]] == "left" and "at_upper_limit_no_tighten" in b["reasons"]}
    right_flagged = {b["rim_hole"] for b in cands[0]["blocked_spokes"]
                     if spokes[b["rim_hole"]] == "right" and "at_upper_limit_no_tighten" in b["reasons"]}
    assert len(left_flagged) == 18 and right_flagged == set()


def test_cumulative_turns_accumulate_across_rounds(plan_id, batch):
    bid = batch["batch_id"]
    holes = _holes(bid)
    body, chosen = _confirm_first_round(bid, _readings(holes, lateral=0.5))
    fr1 = body["round"]["cumulative_turns"]
    assert fr1, "第一轮动作应产生累计转动量"
    client.post(f"/batches/{bid}/rounds/complete")
    # 第二轮选不动作：累计量保持第一轮值
    post_t = {a["rim_hole"]: a["tension_after_n"] for a in chosen["spoke_actions"]}

    def g(t):
        return 10.0 if t <= 400 else (30.0 if t >= 1500 else 10 + 20 * (t - 400) / 1100)

    rd2 = [{"rim_hole": h, "gauge_reading": g(post_t.get(h, 900.0)),
            "radial_mm": 0.0, "lateral_mm": chosen["metrics"]["dish_offset_mm"]}
           for h in holes]
    body2, _ = _confirm_first_round(bid, rd2, candidate_index=0)
    assert body2["round"]["cumulative_turns"] == fr1
    # 定稿轨迹保留每轮累计值与来源版本
    client.post(f"/batches/{bid}/rounds/complete")
    client.post(f"/batches/{bid}/finalize")
    tr = client.get(f"/batches/{bid}/trajectory").json()
    assert tr["trajectory"]["rounds"][0]["cumulative_turns"] == fr1
    assert tr["plan_ref"]["version"] == 1


def test_calibration_interpolation_and_range():
    curve = tuning.validate_curve([(10, 400), (20, 900), (30, 1500)])
    assert tuning.tension_from_reading(25, curve) == 1200.0
    assert tuning.tension_from_reading(9.9, curve) is None
    assert tuning.tension_from_reading(30.1, curve) is None
    with pytest.raises(Exception):
        tuning.validate_curve([(10, 400)])
