"""端到端与几何测试。"""

import json
import math
import os

_TMP_DB = "/tmp/test_wheel_api.db"
if os.path.exists(_TMP_DB):
    os.remove(_TMP_DB)
os.environ["WHEEL_API_DB"] = _TMP_DB

import pytest
from fastapi.testclient import TestClient

from wheel_api.app import app

client = TestClient(app)

SPEC = {
    "name": "test-wheel",
    "rim": {
        "erd_mm": 600.0,
        "holes": 36,
        "hole_offset_left_mm": 1.5,
        "hole_offset_right_mm": 1.5,
        "valve_position": 35,
    },
    "hub": {
        "holes_per_flange": 18,
        "flange_pcd_left_mm": 58.0,
        "flange_pcd_right_mm": 45.0,
        "center_to_flange_left_mm": 35.0,
        "center_to_flange_right_mm": 20.0,
        "spoke_hole_diameter_mm": 2.4,
    },
    "left": {"cross": 3, "heads_in": "trailing"},
    "right": {"cross": 3, "heads_in": "trailing"},
    "spoke_diameter_mm": 2.0,
}


def _spec(**overrides):
    import copy

    s = copy.deepcopy(SPEC)
    for k, v in overrides.items():
        s[k] = v
    return s


def test_create_plan_and_immutable_reads():
    r = client.post("/plans", json=SPEC)
    assert r.status_code == 201, r.text
    body = r.json()
    assert body["version"] == 1
    assert body["formula_version"]
    assert "spoke_length" in body["formulas"]
    assert len(body["geometry"]["per_hole"]) == 36
    assert len(body["lacing"]["mapping"]) == 36
    assert len(body["lacing"]["sequence"]) == 36
    assert body["svg"].startswith("<svg")
    assert body["inputs"]["rim"]["erd_mm"] == 600.0

    pid = body["plan_id"]
    r1 = client.get(f"/plans/{pid}/versions/1")
    r2 = client.get(f"/plans/{pid}/versions/1")
    assert r1.status_code == 200 and r2.status_code == 200
    assert r1.content == r2.content  # 同一版本重复读取逐字节一致
    assert r1.json() == body


def test_geometry_values_radial_symmetric():
    spec = _spec(
        rim={"erd_mm": 600.0, "holes": 36, "valve_position": 35},
        hub={
            "holes_per_flange": 18,
            "flange_pcd_left_mm": 50.0,
            "flange_pcd_right_mm": 50.0,
            "center_to_flange_left_mm": 30.0,
            "center_to_flange_right_mm": 30.0,
            "spoke_hole_diameter_mm": 2.4,
        },
        left={"cross": 0},
        right={"cross": 0},
    )
    r = client.post("/plans", json=spec)
    assert r.status_code == 201, r.text
    g = r.json()["geometry"]
    # 径向、对称：L = sqrt(R^2 + r^2 + w^2 - 2Rr) - s/2
    expected = math.sqrt(300.0**2 + 25.0**2 + 30.0**2 - 2 * 300.0 * 25.0) - 1.2
    assert g["sides"]["right"]["spoke_length_mm"] == pytest.approx(expected, abs=1e-3)
    assert g["tension_ratio_left_to_right"] == pytest.approx(1.0, abs=1e-3)
    # 径向时出线角 = 90 度（与切线垂直）
    hole0 = next(h for h in g["per_hole"] if h["rim_hole"] == 0)
    assert hole0["flange_exit_angle_deg"] == pytest.approx(90.0, abs=1e-3)


def test_hole_count_mismatch():
    spec = _spec(hub={**SPEC["hub"], "holes_per_flange": 16})
    r = client.post("/plans", json=spec)
    assert r.status_code == 400
    err = r.json()["error"]
    assert err["code"] == "HOLE_COUNT_MISMATCH"
    assert err["details"]["rim_holes"] == 36
    assert err["details"]["hub_holes_total"] == 32


def test_hole_count_not_divisible_by_4():
    spec = _spec(
        rim={**SPEC["rim"], "holes": 34},
        hub={**SPEC["hub"], "holes_per_flange": 17},
    )
    r = client.post("/plans", json=spec)
    assert r.status_code == 400
    assert r.json()["error"]["code"] == "HOLE_COUNT_INVALID"


def test_cross_infeasible():
    spec = _spec(right={"cross": 9, "heads_in": "trailing"})
    r = client.post("/plans", json=spec)
    assert r.status_code == 400
    err = r.json()["error"]
    assert err["code"] == "CROSS_INFEASIBLE"
    assert err["details"]["side"] == "right"
    assert err["details"]["cross"] == 9
    assert err["details"]["max_cross"] == 4  # (18-2)//4


def test_duplicate_rim_hole_mapping():
    override = [{"side": "right", "rim_hole": 0, "hub_hole": 0}] * 2
    spec = _spec(mapping_override=override)
    r = client.post("/plans", json=spec)
    assert r.status_code == 400
    err = r.json()["error"]
    assert err["code"] == "DUPLICATE_HOLE_MAPPING"
    assert err["details"]["rim_hole"] == 0


def test_duplicate_hub_hole_mapping():
    # 圈孔互不重复，但右侧法兰孔 3 被两根辐条占用
    override = []
    for m in range(18):
        override.append({"side": "right", "rim_hole": 2 * m, "hub_hole": 3 if m < 2 else m})
    for m in range(18):
        override.append({"side": "left", "rim_hole": 2 * m + 1, "hub_hole": m})
    spec = _spec(mapping_override=override)
    r = client.post("/plans", json=spec)
    assert r.status_code == 400
    err = r.json()["error"]
    assert err["code"] == "DUPLICATE_HOLE_MAPPING"
    assert err["details"]["side"] == "right"
    assert err["details"]["hub_hole"] == 3


def test_valve_avoidance_and_first_spoke():
    r = client.post("/plans", json=SPEC)
    lac = r.json()["lacing"]
    assert lac["phase"] in (0, 1)
    assert lac["valve"]["clearance_mm"] > 0
    assert lac["first_spoke"]["rim_hole"] == 0
    assert lac["valve"]["position_between"] == [35, 0]
    # 编轮次序覆盖全部 36 根且步骤连续
    steps = [s["step"] for s in lac["sequence"]]
    assert steps == list(range(1, 37))


def test_optimizer_flow_and_versioning():
    pid = client.post("/plans", json=SPEC).json()["plan_id"]
    latest = client.get(f"/plans/{pid}").json()
    tl = latest["geometry"]["sides"]["left"]["spoke_length_mm"]
    tr = latest["geometry"]["sides"]["right"]["spoke_length_mm"]
    inv = [
        {"length_mm": round(tl), "count": 24},
        {"length_mm": round(tl) + 1.0},
        {"length_mm": round(tr)},
        {"length_mm": round(tr) - 1.0},
    ]
    opt = {
        "inventory": inv,
        "washers_mm": [0.5, 1.0],
        "tension_min_n": 600.0,
        "tension_max_n": 1400.0,
        "length_tolerance_mm": 1.0,
        "limit": 5,
    }
    r = client.post(f"/plans/{pid}/optimize", json=opt)
    assert r.status_code == 201, r.text
    body = r.json()
    assert body["version"] == 2
    result = body["optimization"]
    assert result["feasible"] is True
    combos = result["combos"]
    assert combos, "应至少有一个合格组合"
    devs = [c["max_deviation_mm"] for c in combos]
    assert devs == sorted(devs)  # 按最大长度偏差升序
    assert all(d <= 1.0 for d in devs)
    assert combos[0]["rank"] == 1
    assert all(c["spoke_spec_count"] >= 1 for c in combos)
    assert result["tension"]["margin_n"] > 0

    # 版本 1 不含优化结果且内容不变
    v1a = client.get(f"/plans/{pid}/versions/1").content
    v1b = client.get(f"/plans/{pid}/versions/1").content
    assert v1a == v1b
    assert "optimization" not in json.loads(v1a)
    # 版本 2 重复读取一致
    v2a = client.get(f"/plans/{pid}/versions/2").content
    v2b = client.get(f"/plans/{pid}/versions/2").content
    assert v2a == v2b == r.content


def test_optimizer_excludes_bad_engagement():
    pid = client.post("/plans", json=SPEC).json()["plan_id"]
    latest = client.get(f"/plans/{pid}").json()
    tr = latest["geometry"]["sides"]["right"]["spoke_length_mm"]
    tl = latest["geometry"]["sides"]["left"]["spoke_length_mm"]
    # 只提供过短的辐条：螺纹啮合不足，全部排除
    opt = {
        "inventory": [{"length_mm": tr - 5.0}, {"length_mm": tl - 5.0}],
        "tension_min_n": 600.0,
        "tension_max_n": 1400.0,
        "length_tolerance_mm": 6.0,
        "min_thread_engagement_mm": 6.0,
        "spoke_thread_length_mm": 9.0,
    }
    r = client.post(f"/plans/{pid}/optimize", json=opt)
    assert r.status_code == 201
    result = r.json()["optimization"]
    assert result["combos"] == []
    assert result["excluded"]["thread_engagement"] > 0


def test_optimizer_tension_infeasible():
    pid = client.post("/plans", json=SPEC).json()["plan_id"]
    opt = {
        "inventory": [{"length_mm": 290.0}, {"length_mm": 291.0}],
        "tension_min_n": 1200.0,
        "tension_max_n": 1300.0,
    }
    r = client.post(f"/plans/{pid}/optimize", json=opt)
    assert r.status_code == 201
    result = r.json()["optimization"]
    # 张力比 < 1 时左侧张力 = ratio * 右侧，1200 下限无法满足两侧
    assert result["feasible"] is False
    assert result["combos"] == []


def test_new_wheel_version_and_404():
    pid = client.post("/plans", json=SPEC).json()["plan_id"]
    spec2 = _spec(name="v2", left={"cross": 2, "heads_in": "leading"})
    r = client.post("/plans/{pid}/versions".format(pid=pid), json=spec2)
    assert r.status_code == 201
    assert r.json()["version"] == 2
    assert r.json()["inputs"]["left"]["cross"] == 2
    assert client.get("/plans/whl_nonexistent").status_code == 404
    assert client.get(f"/plans/{pid}/versions/99").status_code == 404


def test_first_spoke_follows_valve():
    # 阀孔移到 10 与 11 号圈孔之间：首根应落在 11 号孔，且编轮第 1 步即首根
    spec = _spec(rim={**SPEC["rim"], "valve_position": 10})
    r = client.post("/plans", json=spec)
    assert r.status_code == 201, r.text
    lac = r.json()["lacing"]
    assert lac["valve"]["position_between"] == [10, 11]
    assert lac["first_spoke"]["rim_hole"] == 11
    assert lac["sequence"][0]["rim_hole"] == 11
    assert [s["step"] for s in lac["sequence"]] == list(range(1, 37))
    assert len({(s["side"], s["rim_hole"]) for s in lac["sequence"]}) == 36


def test_optimizer_per_hole_tolerance():
    pid = client.post("/plans", json=SPEC).json()["plan_id"]
    g = client.get(f"/plans/{pid}").json()["geometry"]
    tl = g["sides"]["left"]["spoke_length_mm"]
    tr = g["sides"]["right"]["spoke_length_mm"]
    opt = {
        "inventory": [{"length_mm": tl}, {"length_mm": tr}],
        "tension_min_n": 600.0,
        "tension_max_n": 1400.0,
        "length_tolerance_mm": 1.0,
    }
    r = client.post(f"/plans/{pid}/optimize", json=opt)
    combos = r.json()["optimization"]["combos"]
    assert combos
    # 逐孔判定：内/外穿修正使各孔理想长度相对汇总值散布约 ±0.44mm
    assert combos[0]["max_deviation_mm"] == pytest.approx(0.441, abs=0.02)
    # 容差收紧到 0.01mm：逐孔最大偏差 0.441mm 超出容差，应全部排除
    opt["length_tolerance_mm"] = 0.01
    r2 = client.post(f"/plans/{pid}/optimize", json=opt)
    res2 = r2.json()["optimization"]
    assert res2["combos"] == []
    assert res2["excluded"]["length_tolerance"] > 0


def test_optimizer_merges_inventory_counts():
    # 对称轮组：两侧理想长度一致，单一长度即可覆盖两侧 36 根
    spec = _spec(
        rim={"erd_mm": 600.0, "holes": 36, "valve_position": 35},
        hub={
            "holes_per_flange": 18,
            "flange_pcd_left_mm": 50.0,
            "flange_pcd_right_mm": 50.0,
            "center_to_flange_left_mm": 30.0,
            "center_to_flange_right_mm": 30.0,
            "spoke_hole_diameter_mm": 2.4,
        },
        left={"cross": 3},
        right={"cross": 3},
    )
    pid = client.post("/plans", json=spec).json()["plan_id"]
    g = client.get(f"/plans/{pid}").json()["geometry"]
    ideal = g["sides"]["right"]["spoke_length_mm"]
    l0 = round(ideal)
    opt = {
        # 两行相同长度、各 18 根：合并后 36 根恰好满足
        "inventory": [{"length_mm": l0, "count": 18}, {"length_mm": l0, "count": 18}],
        "tension_min_n": 600.0,
        "tension_max_n": 1400.0,
        "length_tolerance_mm": 1.0,
    }
    r = client.post(f"/plans/{pid}/optimize", json=opt)
    res = r.json()["optimization"]
    assert res["feasible"] is True
    assert res["combos"], "同长度分行库存合并后应凑足 36 根"
    c = res["combos"][0]
    assert c["left"]["spoke_length_mm"] == l0
    assert c["right"]["spoke_length_mm"] == l0
    assert c["spoke_spec_count"] == 1
