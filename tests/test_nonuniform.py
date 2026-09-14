"""非等距钻孔轮圈 / 成对辐条 / 法兰相位与逐孔角度的端到端测试。"""

import copy
import math
import os

_TMP_DB = "/tmp/test_wheel_api_nonuniform.db"
if os.path.exists(_TMP_DB):
    os.remove(_TMP_DB)
os.environ["WHEEL_API_DB"] = _TMP_DB

import pytest
from fastapi.testclient import TestClient

from wheel_api import storage
from wheel_api.app import app

client = TestClient(app)

HUB = {
    "holes_per_flange": 18,
    "flange_pcd_left_mm": 50.0,
    "flange_pcd_right_mm": 50.0,
    "center_to_flange_left_mm": 30.0,
    "center_to_flange_right_mm": 30.0,
    "spoke_hole_diameter_mm": 2.4,
}


def _paired_table(pairs=18, gap_deg=6.0, wander_deg=2.0, offset=1.5):
    """成对钻孔：每对一左一右、对内间隔 gap_deg；对中心交替摆动使每侧孔距非等距。"""
    table = []
    for m in range(pairs):
        center = m * 360.0 / pairs + (wander_deg if m % 2 else -wander_deg)
        table.append({
            "id": 2 * m,
            "angle_deg": (center - gap_deg / 2.0) % 360.0,
            "side": "right",
            "axial_offset_mm": offset,
        })
        table.append({
            "id": 2 * m + 1,
            "angle_deg": (center + gap_deg / 2.0) % 360.0,
            "side": "left",
            "axial_offset_mm": offset,
        })
    return table


def _paired_spec(**overrides):
    spec = {
        "name": "paired-wheel",
        "rim": {
            "erd_mm": 600.0,
            "holes": 36,
            "valve_position": 35,
            "hole_table": _paired_table(),
        },
        "hub": copy.deepcopy(HUB),
        "left": {"cross": 3, "heads_in": "trailing"},
        "right": {"cross": 3, "heads_in": "trailing"},
        "spoke_diameter_mm": 2.0,
    }
    spec.update(overrides)
    return spec


def _standard_spec(**overrides):
    spec = {
        "name": "std-wheel",
        "rim": {"erd_mm": 600.0, "holes": 36, "valve_position": 35},
        "hub": copy.deepcopy(HUB),
        "left": {"cross": 3, "heads_in": "trailing"},
        "right": {"cross": 3, "heads_in": "trailing"},
        "spoke_diameter_mm": 2.0,
    }
    spec.update(overrides)
    return spec


def test_paired_rim_uses_actual_angles():
    r = client.post("/plans", json=_paired_spec())
    assert r.status_code == 201, r.text
    body = r.json()
    geo = body["geometry"]
    # 角度来源：轮圈孔表 + 花鼓默认相位
    assert geo["angle_source"] == {
        "rim": "hole_table",
        "hub_left": "default_half_pitch",
        "hub_right": "default_zero",
    }
    # 逐孔角度与孔表一致（不再由孔号推导）
    table = {h["id"]: h for h in _paired_table()}
    for h in geo["per_hole"]:
        assert h["rim_angle_deg"] == pytest.approx(table[h["rim_hole"]]["angle_deg"] % 360.0, abs=1e-3)
    # 非等距孔位下同侧逐孔长度存在散布
    right_lengths = [h["length_mm"] for h in geo["per_hole"] if h["side"] == "right"]
    assert max(right_lengths) - min(right_lengths) > 0.05
    # 映射为双射且覆盖全部 36 孔
    mapping = body["lacing"]["mapping"]
    assert len(mapping) == 36
    assert len({e["rim_hole"] for e in mapping}) == 36
    assert body["lacing"]["flange_shift"] is not None
    # 快照重复读取逐字节一致
    pid = body["plan_id"]
    r1 = client.get(f"/plans/{pid}/versions/1")
    r2 = client.get(f"/plans/{pid}/versions/1")
    assert r1.status_code == 200 and r1.content == r2.content


def test_paired_rim_deterministic_search():
    """同一非等距方案重复提交，自动穿法搜索结果一致。"""
    b1 = client.post("/plans", json=_paired_spec()).json()
    b2 = client.post("/plans", json=_paired_spec()).json()
    assert b1["lacing"] == b2["lacing"]
    assert b1["geometry"] == b2["geometry"]


def test_paired_rim_first_spoke_and_sequence():
    body = client.post("/plans", json=_paired_spec()).json()
    lac = body["lacing"]
    # 阀孔在 35 号孔（345°）与角序下一孔 0 号（355°）之间，首根为 0 号
    assert lac["valve"]["position_between"] == [35, 0]
    assert lac["first_spoke"]["rim_hole"] == 0
    assert lac["valve"]["clearance_mm"] > 0
    seq = lac["sequence"]
    assert [s["step"] for s in seq] == list(range(1, 37))
    assert seq[0]["rim_hole"] == 0  # 第 1 步即首根
    assert body["svg"].startswith("<svg") and "VALVE" in body["svg"]


def test_hub_flange_phase_and_explicit_angles():
    # 起始相位
    spec = _standard_spec(hub={**HUB, "flange_phase_right_deg": 5.0})
    body = client.post("/plans", json=spec).json()
    geo = body["geometry"]
    assert geo["angle_source"]["hub_right"] == "phase"
    for h in geo["per_hole"]:
        if h["side"] == "right":
            assert h["hub_angle_deg"] == pytest.approx((5.0 + 20.0 * h["hub_hole"]) % 360.0, abs=1e-3)
    # 逐孔角度
    angles_left = [(j * 20.0 + 7.0) % 360.0 for j in range(18)]
    spec = _standard_spec(hub={**HUB, "flange_angles_left_deg": angles_left})
    body = client.post("/plans", json=spec).json()
    geo = body["geometry"]
    assert geo["angle_source"]["hub_left"] == "explicit"
    for h in geo["per_hole"]:
        if h["side"] == "left":
            assert h["hub_angle_deg"] == pytest.approx(angles_left[h["hub_hole"]], abs=1e-3)


def test_hole_table_validation_422():
    # 编号重复
    tab = _paired_table()
    tab[1]["id"] = tab[0]["id"]
    r = client.post("/plans", json=_paired_spec(rim={**_paired_spec()["rim"], "hole_table": tab}))
    assert r.status_code == 422
    # 覆盖不足
    r = client.post("/plans", json=_paired_spec(rim={**_paired_spec()["rim"], "hole_table": _paired_table()[:-2]}))
    assert r.status_code == 422
    # 角度重复
    tab = _paired_table()
    tab[3]["angle_deg"] = tab[2]["angle_deg"]
    r = client.post("/plans", json=_paired_spec(rim={**_paired_spec()["rim"], "hole_table": tab}))
    assert r.status_code == 422
    # 侧别数量不等
    tab = _paired_table()
    tab[1]["side"] = "right"
    r = client.post("/plans", json=_paired_spec(rim={**_paired_spec()["rim"], "hole_table": tab}))
    assert r.status_code == 422


def test_hub_angle_validation_422():
    # 相位与逐孔角度互斥
    r = client.post("/plans", json=_standard_spec(
        hub={**HUB, "flange_phase_right_deg": 5.0,
             "flange_angles_right_deg": [j * 20.0 for j in range(18)]}))
    assert r.status_code == 422
    # 逐孔角度覆盖不足
    r = client.post("/plans", json=_standard_spec(
        hub={**HUB, "flange_angles_left_deg": [j * 20.0 for j in range(16)]}))
    assert r.status_code == 422
    # 逐孔角度重复
    dup = [j * 20.0 for j in range(18)]
    dup[5] = dup[4]
    r = client.post("/plans", json=_standard_spec(hub={**HUB, "flange_angles_left_deg": dup}))
    assert r.status_code == 422


def test_lacing_infeasible_returns_conflicting_holes():
    # 右法兰孔位挤在 10° 内：任何双射都无法满足 3x 交叉数
    hub = {**HUB, "flange_angles_right_deg": [j * 0.6 for j in range(18)]}
    r = client.post("/plans", json=_standard_spec(hub=hub))
    assert r.status_code == 400, r.text
    err = r.json()["error"]
    assert err["code"] == "LACING_INFEASIBLE"
    conflicts = err["details"]["conflicts"]
    assert conflicts
    assert all(c["reason"] in ("cross_direction", "cross_count") for c in conflicts)
    assert any(c["side"] == "right" for c in conflicts)
    assert all("rim_hole" in c and "hub_hole" in c for c in conflicts)


def test_valve_clearance_infeasible():
    spec = _standard_spec(valve_clearance_min_mm=200.0)
    r = client.post("/plans", json=spec)
    assert r.status_code == 400, r.text
    err = r.json()["error"]
    assert err["code"] == "LACING_INFEASIBLE"
    assert err["details"]["reason"] == "valve_clearance"
    assert err["details"]["valve_clearance_min_mm"] == 200.0
    assert err["details"]["best_clearance_mm"] < 200.0
    assert err["details"]["conflicts"]  # 距阀孔最近的冲突孔


def test_mapping_override_with_hole_table():
    # 未知圈孔编号
    override = [{"side": "right", "rim_hole": 999, "hub_hole": 0}]
    r = client.post("/plans", json=_paired_spec(mapping_override=override))
    assert r.status_code == 400
    assert r.json()["error"]["code"] == "MAPPING_INVALID"
    # 圈孔重复占用
    override = [{"side": "right", "rim_hole": 0, "hub_hole": 0}] * 2
    r = client.post("/plans", json=_paired_spec(mapping_override=override))
    assert r.status_code == 400
    assert r.json()["error"]["code"] == "DUPLICATE_HOLE_MAPPING"
    # 侧别不符（0 号孔在右侧）
    override = [{"side": "left", "rim_hole": 0, "hub_hole": 0}]
    r = client.post("/plans", json=_paired_spec(mapping_override=override))
    assert r.status_code == 400
    assert r.json()["error"]["code"] == "MAPPING_INVALID"
    # 完整自定义映射：取自动搜索结果原样回灌，结果一致
    auto = client.post("/plans", json=_paired_spec()).json()
    override = [
        {"side": e["side"], "rim_hole": e["rim_hole"], "hub_hole": e["hub_hole"]}
        for e in auto["lacing"]["mapping"]
    ]
    r = client.post("/plans", json=_paired_spec(mapping_override=override))
    assert r.status_code == 201, r.text
    manual = r.json()
    assert manual["lacing"]["mapping"] == auto["lacing"]["mapping"]
    assert manual["geometry"]["per_hole"] == auto["geometry"]["per_hole"]


def test_optimizer_on_nonuniform_geometry():
    pid = client.post("/plans", json=_paired_spec()).json()["plan_id"]
    geo = client.get(f"/plans/{pid}").json()["geometry"]
    ideals = [h["length_mm"] for h in geo["per_hole"]]
    mid = (min(ideals) + max(ideals)) / 2.0
    spread = (max(ideals) - min(ideals)) / 2.0
    opt = {
        "inventory": [{"length_mm": round(mid, 1)}],
        "tension_min_n": 600.0,
        "tension_max_n": 1400.0,
        "length_tolerance_mm": spread + 0.2,
        "max_protrusion_mm": spread + 0.2,
        "min_thread_engagement_mm": 5.0,
    }
    r = client.post(f"/plans/{pid}/optimize", json=opt)
    assert r.status_code == 201, r.text
    result = r.json()["optimization"]
    assert result["feasible"] is True
    assert result["combos"], "非等距逐孔长度下库存优化应给出合格组合"
    # 逐孔偏差区间随组合给出
    assert "deviation_range_mm" in result["combos"][0]["left"]


def test_legacy_snapshot_still_readable():
    """既有（1.1 时代、无 angle_source 等新字段）标准方案快照仍能原样读取。"""
    plan_id, _ = storage.create_plan("legacy-std")
    legacy = {
        "plan_id": plan_id,
        "version": 1,
        "created_at": "2026-09-13T00:00:00+00:00",
        "formula_version": "wheel-geometry/1.1",
        "formulas": {"spoke_length": "L = ..."},
        "inputs": {"rim": {"erd_mm": 600.0, "holes": 36}, "hub": {}},
        "geometry": {"holes_total": 36, "per_hole": []},
        "lacing": {"mapping": []},
        "svg": "<svg></svg>",
    }
    text = storage.canonical(legacy)
    storage.insert_version(plan_id, 1, text)
    r1 = client.get(f"/plans/{plan_id}")
    r2 = client.get(f"/plans/{plan_id}/versions/1")
    assert r1.status_code == 200 and r2.status_code == 200
    assert r1.content == r2.content == text.encode()


def test_uniform_plan_angle_source_and_angles():
    """标准等距方案：角度来源标记为 uniform，逐孔角度与孔号推导一致。"""
    body = client.post("/plans", json=_standard_spec()).json()
    geo = body["geometry"]
    assert geo["angle_source"]["rim"] == "uniform"
    for h in geo["per_hole"]:
        assert h["rim_angle_deg"] == pytest.approx(360.0 * h["rim_hole"] / 36.0, abs=1e-3)
