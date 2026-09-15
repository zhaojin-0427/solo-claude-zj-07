"""服役载荷校核单端到端测试。

覆盖：不可变版本绑定、Pydantic 来源/单位/角度/载荷校验、逐角度平衡解、
失张退出重解、张力包络/首根松弛/最小余量/最危险角度、方程无解指工况、
预张力搜索排序与锁定、草拟/已采用流转、冻结一致与来源不改写。
"""

import json
import math
import os

_TMP_DB = "/tmp/test_service_load.db"
if os.path.exists(_TMP_DB):
    os.remove(_TMP_DB)
os.environ["WHEEL_API_DB"] = _TMP_DB

import pytest
from fastapi.testclient import TestClient

from wheel_api.app import app
from wheel_api import service_load

client = TestClient(app)

SPEC = {
    "name": "service-wheel",
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

SWEEP_30 = {"start_deg": 0.0, "end_deg": 360.0, "step_deg": 30.0}
SWEEP_45 = {"start_deg": 0.0, "end_deg": 360.0, "step_deg": 45.0}


@pytest.fixture(scope="module")
def plan_id():
    r = client.post("/plans", json=SPEC)
    assert r.status_code == 201, r.text
    return r.json()["plan_id"]


# 对称轮：左右法兰相同，等张力自平衡（零载下增量位移为零）
SYMMETRIC_SPEC = {
    "name": "symmetric-wheel",
    "rim": {"erd_mm": 600.0, "holes": 36, "valve_position": 35},
    "hub": {
        "holes_per_flange": 18,
        "flange_pcd_left_mm": 50.0,
        "flange_pcd_right_mm": 50.0,
        "center_to_flange_left_mm": 30.0,
        "center_to_flange_right_mm": 30.0,
    },
    "left": {"cross": 3},
    "right": {"cross": 3},
}


@pytest.fixture(scope="module")
def sym_plan_id():
    r = client.post("/plans", json=SYMMETRIC_SPEC)
    assert r.status_code == 201, r.text
    return r.json()["plan_id"]


def _uniform(left=900.0, right=900.0):
    return {"type": "uniform", "left_n": left, "right_n": right}


def _sheet(plan_id, *, tension_source=None, sweep=SWEEP_30, loads=None,
           tmin=300.0, tmax=1500.0, tol=1e-9, search=None, version=None,
           name="sheet"):
    body = {
        "name": name,
        "tension_source": tension_source or _uniform(),
        "sweep": sweep,
        "loads": loads or {"radial_n": 900.0, "lateral_n": 0.0,
                           "torque_nm": 0.0, "load_factor": 1.0},
        "tension_min_n": tmin,
        "tension_max_n": tmax,
        "residual_tolerance": tol,
    }
    if search is not None:
        body["pretension_search"] = search
    url = f"/plans/{plan_id}/load-sheets"
    if version is not None:
        url += f"?version={version}"
    return client.post(url, json=body)


# ---------------------------------------------------------------------------
# 创建、取数与基本力学
# ---------------------------------------------------------------------------

def test_create_sheet_pins_plan_version(plan_id):
    r = _sheet(plan_id)
    assert r.status_code == 201, r.text
    d = r.json()["load_sheet"]
    assert d["status"] == "draft"
    assert d["plan_ref"] == {"plan_id": plan_id, "version": 1,
                             "formula_version": "wheel-geometry/1.2"}
    assert d["service_formula_version"] == "wheel-service-load/1.0"
    assert set(d["service_formulas"]) >= {"equilibrium", "slack_release",
                                          "axial_stiffness", "ranking"}
    # 逐孔来源：孔序与数量，左右名义预张力
    assert len(d["pretension"]) == 36
    side_of = {p["rim_hole"]: p["side"] for p in d["pretension"]}
    assert all(p["pretension_n"] == 900.0 for p in d["pretension"])
    assert d["tension_source"] == {"type": "uniform", "left_n": 900.0,
                                   "right_n": 900.0}
    # 角度覆盖：0..360 含端点共 13 个
    assert d["case"]["angles"]["angle_count"] == 13
    assert d["case"]["angles"]["angles_deg"][0] == 0.0
    assert d["case"]["angles"]["angles_deg"][-1] == 360.0
    # 单位记录
    units = d["case"]["loads"]["units"]
    assert units["radial_n"] == "N" and units["torque_nm"].startswith("N·m")


def test_per_angle_equilibrium_and_displacement(plan_id):
    r = _sheet(plan_id, tension_source=_uniform(800.0, 1000.0),
               loads={"radial_n": 900.0, "lateral_n": 100.0,
                      "torque_nm": 30.0, "torque_direction": "drive"})
    assert r.status_code == 201, r.text
    res = r.json()["load_sheet"]["result"]
    assert len(res["per_angle"]) == 13
    for a in res["per_angle"]:
        # 平衡残差在容差内（数值零）
        assert a["residual"]["force_rel"] < 1e-9
        assert a["residual"]["moment_rel"] < 1e-9
        disp = a["hub_displacement_mm"]
        assert set(disp) == {"x", "y", "z"}
        # 每根辐条都有非负张力
        for t in a["spoke_tensions_n"]:
            assert t["tension_n"] > 0 and t["active"] is True
    # 载荷角 0° 时花鼓沿 +x 移动（向受载辐条侧）；扭矩产生转动
    a0 = res["per_angle"][0]
    assert a0["hub_displacement_mm"]["x"] > 0.0
    assert a0["hub_displacement_mm"]["y"] == pytest.approx(0.0, abs=1e-9)
    assert a0["hub_rotation_deg"] != 0.0


def test_drive_and_brake_torque_opposite_sign(plan_id):
    kw = dict(tension_source=_uniform(), tmin=0.0, tmax=2000.0,
              sweep={"start_deg": 0.0, "end_deg": 90.0, "step_deg": 90.0})
    rd = _sheet(plan_id, loads={"radial_n": 0.0, "torque_nm": 50.0,
                                "torque_direction": "drive"}, **kw).json()
    rb = _sheet(plan_id, loads={"radial_n": 0.0, "torque_nm": 50.0,
                                "torque_direction": "brake"}, **kw).json()
    qd = rd["load_sheet"]["result"]["per_angle"][0]["hub_rotation_deg"]
    qb = rb["load_sheet"]["result"]["per_angle"][0]["hub_rotation_deg"]
    assert qd == pytest.approx(-qb, abs=1e-9) and abs(qd) > 1e-6
    # 扭矩单位换算：factored torque N·mm = 50 * 1000
    assert rd["load_sheet"]["case"]["factored_loads"]["torque_nmm"] == 50_000.0
    assert rb["load_sheet"]["case"]["factored_loads"]["torque_nmm"] == -50_000.0


def test_load_factor_multiplies_all_loads(plan_id):
    r = _sheet(plan_id, loads={"radial_n": 1000.0, "lateral_n": 100.0,
                               "torque_nm": 40.0, "load_factor": 1.5})
    fl = r.json()["load_sheet"]["case"]["factored_loads"]
    assert fl["radial_n"] == 1500.0
    assert fl["lateral_n"] == 150.0
    assert fl["torque_nmm"] == 60_000.0


# ---------------------------------------------------------------------------
# 失张退出重解
# ---------------------------------------------------------------------------

def test_slack_spokes_release_and_resolve(plan_id):
    r = _sheet(plan_id, tension_source=_uniform(500.0, 500.0),
               loads={"radial_n": 7000.0}, sweep=SWEEP_45, tmin=0.0, tmax=2000.0)
    assert r.status_code == 201, r.text
    res = r.json()["load_sheet"]["result"]
    assert res["slack_spoke_angle_pairs"] > 0
    assert res["violation_count"] > 0
    # 失张辐条张力置 0、active=False，且重新求解后残差仍为零
    for a in res["per_angle"]:
        assert a["residual"]["force_rel"] < 1e-9
        assert a["residual"]["moment_rel"] < 1e-9
        for t in a["spoke_tensions_n"]:
            assert t["tension_n"] >= 0.0
            if not t["active"]:
                assert t["tension_n"] == 0.0
                assert t["rim_hole"] in a["slack_rim_holes"]
        for v in a["violations"]:
            if v["type"] == "slack":
                assert v["tension_n"] == 0.0
    # 首根松弛沿扫描方向（角度最小者）
    assert res["first_event"]["type"] == "slack"
    first_angle = res["first_event"]["angle_deg"]
    slack_angles = [a["angle_deg"] for a in res["per_angle"] if a["slack_rim_holes"]]
    assert first_angle == min(slack_angles)


def test_overload_detection(plan_id):
    r = _sheet(plan_id, loads={"radial_n": 0.0, "torque_nm": 60.0}, tmax=1000.0)
    assert r.status_code == 201, r.text
    res = r.json()["load_sheet"]["result"]
    assert res["violation_count"] > 0
    assert res["first_event"]["type"] == "over_max"
    # 包络最大值确实超上限
    assert res["tension_envelope"]["global"]["max_n"] > 1000.0
    # 最危险角度携带违规清单
    wa = res["worst_angle"]
    assert wa["violating_spoke_count"] >= 1
    assert any(v["type"] == "over_max" for v in wa["rim_holes"])


def test_below_min_distinct_from_slack(plan_id):
    # 下限 400 但辐条未物理失张（张力仍为正）：below_min，不是 slack
    r = _sheet(plan_id, tension_source=_uniform(500.0, 500.0),
               loads={"radial_n": 4500.0}, sweep=SWEEP_45, tmin=400.0, tmax=2000.0)
    assert r.status_code == 201, r.text
    res = r.json()["load_sheet"]["result"]
    types = {v["type"] for a in res["per_angle"] for v in a["violations"]}
    assert "below_min" in types
    assert res["slack_spoke_angle_pairs"] == 0  # 物理失张为 0
    assert res["first_event"]["type"] == "below_min"


# ---------------------------------------------------------------------------
# 包络 / 最小余量 / 最危险角度
# ---------------------------------------------------------------------------

def test_envelope_minimum_margin_and_worst_angle(plan_id):
    r = _sheet(plan_id, tension_source=_uniform(900.0, 900.0),
               loads={"radial_n": 900.0})
    res = r.json()["load_sheet"]["result"]
    env = res["tension_envelope"]
    # 逐孔包络与全局包络一致
    gmin = min(p["min_n"] for p in env["per_spoke"])
    gmax = max(p["max_n"] for p in env["per_spoke"])
    assert env["global"]["min_n"] == gmin
    assert env["global"]["max_n"] == gmax
    assert env["global"]["min_rim_hole"] in {p["rim_hole"] for p in env["per_spoke"]}
    # 逐孔余量与最小/最大张力自洽
    for p in env["per_spoke"]:
        assert p["margin_to_min_n"] == pytest.approx(p["min_n"] - 300.0, abs=1e-6)
        assert p["margin_to_max_n"] == pytest.approx(1500.0 - p["max_n"], abs=1e-6)
    mm = res["minimum_margin"]
    assert mm["kind"] in ("to_min", "to_max")
    assert mm["margin_n"] == min(
        min(p["margin_to_min_n"], p["margin_to_max_n"]) for p in env["per_spoke"])
    # 无违规时最危险角度仍给出（0 违规），worst 角有定义
    assert res["worst_angle"]["violating_spoke_count"] == 0
    assert 0.0 <= res["worst_angle"]["angle_deg"] <= 360.0


# ---------------------------------------------------------------------------
# 方程无解 / 残差超限：指出工况角度
# ---------------------------------------------------------------------------

def test_radial_lacing_with_torque_is_infeasible_and_points_angle(plan_id):
    # 径向穿法（cross=0）辐条全部通过轴心，对绕轴转动无刚度 -> K 奇异
    spec = {**SPEC, "left": {"cross": 0}, "right": {"cross": 0}}
    pid = client.post("/plans", json=spec).json()["plan_id"]
    r = _sheet(pid, loads={"radial_n": 0.0, "torque_nm": 30.0},
               sweep={"start_deg": 0.0, "end_deg": 90.0, "step_deg": 30.0})
    assert r.status_code == 400
    err = r.json()["error"]
    assert err["code"] == "LOAD_CASE_INFEASIBLE"
    assert err["details"]["angle_deg"] == 0.0
    assert "无解" in err["message"]


def test_radial_lacing_without_torque_solves_with_zero_rotation():
    # 同样的径向穿法、无扭矩：转动自由度无载（零空间右端为零），
    # φ 自由变量取 0，平动方向正常求解
    spec = {**SPEC, "left": {"cross": 0}, "right": {"cross": 0}}
    pid = client.post("/plans", json=spec).json()["plan_id"]
    r = _sheet(pid, loads={"radial_n": 900.0, "torque_nm": 0.0})
    assert r.status_code == 201, r.text
    res = r.json()["load_sheet"]["result"]
    for a in res["per_angle"]:
        assert abs(a["hub_rotation_deg"]) < 1e-9
        assert a["residual"]["force_rel"] < 1e-9


def test_no_sheet_row_when_infeasible(plan_id):
    before = len(client.get("/load-sheets", params={"plan_id": plan_id}).json()["load_sheets"])
    # 径向 + 扭矩必失败
    spec = {**SPEC, "left": {"cross": 0}, "right": {"cross": 0}}
    pid = client.post("/plans", json=spec).json()["plan_id"]
    r = _sheet(pid, loads={"radial_n": 0.0, "torque_nm": 10.0},
               sweep={"start_deg": 0.0, "end_deg": 30.0, "step_deg": 30.0})
    assert r.status_code == 400
    after = len(client.get("/load-sheets", params={"plan_id": pid}).json()["load_sheets"])
    assert after == 0  # 计算失败不留校核单记录
    assert before >= 0


def test_active_set_collapse_when_too_few_spokes_remain():
    # 成对孔 12 孔径向轮：大径向载荷下大部分辐条失张退出，
    # 剩余承载辐条不足以稳定花鼓 -> 主动集重解后 K 奇异，指出角度
    spec = {
        "name": "paired-radial",
        "rim": {"erd_mm": 600.0, "holes": 12,
                "hole_table": [
                    {"id": i, "angle_deg": a,
                     "side": ("right" if i % 2 == 0 else "left")}
                    for i, a in enumerate(
                        [0, 12, 60, 72, 120, 132, 180, 192, 240, 252, 300, 312])]},
        "hub": {"holes_per_flange": 6, "flange_pcd_left_mm": 50.0,
                "flange_pcd_right_mm": 50.0,
                "center_to_flange_left_mm": 30.0,
                "center_to_flange_right_mm": 30.0},
        "left": {"cross": 0}, "right": {"cross": 0},
    }
    pid = client.post("/plans", json=spec).json()["plan_id"]
    r = _sheet(pid, tension_source=_uniform(900.0, 900.0),
               loads={"radial_n": 800.0},
               sweep={"start_deg": 0.0, "end_deg": 90.0, "step_deg": 30.0})
    assert r.status_code == 400
    err = r.json()["error"]
    assert err["code"] == "LOAD_CASE_INFEASIBLE"
    assert err["details"]["angle_deg"] == 0.0


# ---------------------------------------------------------------------------
# Pydantic 输入校验：来源、角度覆盖、载荷范围
# ---------------------------------------------------------------------------

def test_invalid_tension_source_type(plan_id):
    r = _sheet(plan_id, tension_source={"type": "nominal", "left_n": 900.0,
                                        "right_n": 900.0})
    assert r.status_code == 422


def test_uniform_source_must_be_positive(plan_id):
    r = _sheet(plan_id, tension_source=_uniform(left=0.0))
    assert r.status_code == 422


def test_sweep_step_must_divide_span(plan_id):
    r = _sheet(plan_id, sweep={"start_deg": 0.0, "end_deg": 100.0,
                              "step_deg": 30.0})
    assert r.status_code == 422 and "整数倍" in r.text


def test_sweep_span_out_of_range(plan_id):
    r = _sheet(plan_id, sweep={"start_deg": 10.0, "end_deg": 400.0,
                              "step_deg": 10.0})
    assert r.status_code == 422
    r = _sheet(plan_id, sweep={"start_deg": 30.0, "end_deg": 20.0,
                              "step_deg": 10.0})
    assert r.status_code == 422


def test_sweep_step_must_be_positive(plan_id):
    r = _sheet(plan_id, sweep={"start_deg": 0.0, "end_deg": 360.0,
                              "step_deg": 0.0})
    assert r.status_code == 422


def test_load_ranges(plan_id):
    r = _sheet(plan_id, loads={"radial_n": -1.0})
    assert r.status_code == 422
    r = _sheet(plan_id, loads={"radial_n": 900.0, "load_factor": 0.0})
    assert r.status_code == 422
    r = _sheet(plan_id, loads={"radial_n": 900.0, "lateral_n": 9000.0})
    assert r.status_code == 422


def test_tension_window_validation(plan_id):
    r = _sheet(plan_id, tmin=1200.0, tmax=1200.0)
    assert r.status_code == 422 and "tension_max_n" in r.text


def test_search_step_must_divide_range(plan_id):
    r = _sheet(plan_id, search={"max_adjustment_n": 300.0, "step_n": 70.0})
    assert r.status_code == 422 and "整数倍" in r.text


def test_unknown_locked_hole(plan_id):
    r = _sheet(plan_id, search={"locked_rim_holes": [400],
                                "max_adjustment_n": 300.0, "step_n": 50.0})
    assert r.status_code == 400
    assert r.json()["error"]["code"] == "LOAD_SHEET_HOLE_UNKNOWN"
    assert r.json()["error"]["details"]["rim_holes"] == [400]


# ---------------------------------------------------------------------------
# 预张力搜索：排序、锁定、不改写来源
# ---------------------------------------------------------------------------

def _search_sheet(plan_id, F=4500.0, search=None):
    return _sheet(plan_id, tension_source=_uniform(500.0, 500.0),
                  loads={"radial_n": F}, sweep=SWEEP_45,
                  tmin=100.0, tmax=2000.0,
                  search=search or {"max_adjustment_n": 700.0,
                                    "step_n": 50.0, "limit": 8})


def test_pretension_search_options_ranked(plan_id):
    r = _search_sheet(plan_id)
    assert r.status_code == 201, r.text
    ps = r.json()["load_sheet"]["result"]["pretension_search"]
    opts = ps["options"]
    baseline = next(o for o in opts if o["label"] == "baseline")
    assert baseline["adjustments"] == []
    assert baseline["changed_spoke_count"] == 0
    assert baseline["total_adjustment_n"] == 0.0
    # 基线存在违规，搜索应找到无违规方案
    assert baseline["violation_count"] > 0
    assert any(o["feasible"] for o in opts[1:])
    # 统一四项排序：违规数 -> 最大张力 -> 改动孔数 -> 总调节量
    keys = [(o["violation_count"], o["max_tension_n"],
             o["changed_spoke_count"], o["total_adjustment_n"]) for o in opts]
    assert keys == sorted(keys)
    # 标签唯一
    labels = [o["label"] for o in opts]
    assert len(labels) == len(set(labels))
    # 默认选定第 0 个方案（列表位置）
    assert ps["selected_index"] == 0
    assert ps["selected"]["label"] == opts[0]["label"]
    # 每步为步长整数倍且不超过最大调节量、不越服役窗口
    for o in opts:
        for a in o["adjustments"]:
            assert abs(a["delta_n"]) <= 700.0 + 1e-6
            assert abs(a["delta_n"]) % 50.0 < 1e-6
            assert 100.0 - 1e-6 <= a["to_n"] <= 2000.0 + 1e-6


def test_pretension_search_respects_locks(plan_id):
    r = _search_sheet(plan_id, search={
        "locked_rim_holes": [0, 1, 2], "max_adjustment_n": 700.0,
        "step_n": 50.0, "limit": 8})
    assert r.status_code == 201, r.text
    opts = r.json()["load_sheet"]["result"]["pretension_search"]["options"]
    for o in opts:
        assert {0, 1, 2}.isdisjoint({a["rim_hole"] for a in o["adjustments"]})
    assert r.json()["load_sheet"]["result"]["pretension_search"]["locked_rim_holes"] == [0, 1, 2]


def test_search_does_not_modify_source(plan_id):
    r = _search_sheet(plan_id)
    d = r.json()["load_sheet"]
    # 校核单内来源预张力保持 500，方案在 result.pretension_search 内
    assert all(p["pretension_n"] == 500.0 for p in d["pretension"])
    # 方案版本快照不被改写
    snap = client.get(f"/plans/{plan_id}/versions/1").json()
    assert "pretension" not in snap and "service" not in snap


# ---------------------------------------------------------------------------
# 状态流转：草拟 -> 已采用；冻结、修订、放弃
# ---------------------------------------------------------------------------

def test_adopt_freezes_and_repeatable_read(plan_id):
    r = _search_sheet(plan_id)
    d = r.json()["load_sheet"]
    sid = d["load_sheet_id"]
    opts = d["result"]["pretension_search"]["options"]
    idx = next(i for i, o in enumerate(opts) if o["feasible"])
    r = client.post(f"/load-sheets/{sid}/adopt", json={"selected_index": idx})
    assert r.status_code == 200, r.text
    ad = r.json()["load_sheet"]
    assert ad["status"] == "adopted"
    assert ad["adopted"]["selected_index"] == idx
    assert ad["adopted"]["selected_label"] == opts[idx]["label"]
    assert ad["adopted"]["adjustments"] == opts[idx]["adjustments"]
    assert ad["result"]["pretension_search"]["selected"]["label"] == opts[idx]["label"]
    # 冻结后重复读取逐字节一致
    a = client.get(f"/load-sheets/{sid}").content
    b = client.get(f"/load-sheets/{sid}").content
    assert a == b
    # 已采用拒绝再采用 / 修订 / 删除
    assert client.post(f"/load-sheets/{sid}/adopt").status_code == 400
    assert client.post(f"/load-sheets/{sid}/revisions", json={}).status_code == 400
    assert client.delete(f"/load-sheets/{sid}").status_code == 400


def test_adopt_without_search_has_no_options(plan_id):
    sid = _sheet(plan_id).json()["load_sheet"]["load_sheet_id"]
    r = client.post(f"/load-sheets/{sid}/adopt", json={"selected_index": 1})
    assert r.status_code == 400
    assert r.json()["error"]["code"] == "PRETENSION_OPTION_NOT_FOUND"
    # 无 selected_index 可正常采用
    r = client.post(f"/load-sheets/{sid}/adopt", json={})
    assert r.status_code == 200
    assert r.json()["load_sheet"]["adopted"]["selected_index"] is None


def test_adopt_bad_option_index(plan_id):
    d = _search_sheet(plan_id).json()["load_sheet"]
    sid = d["load_sheet_id"]
    n = len(d["result"]["pretension_search"]["options"])
    r = client.post(f"/load-sheets/{sid}/adopt", json={"selected_index": n + 5})
    assert r.status_code == 400
    assert r.json()["error"]["code"] == "PRETENSION_OPTION_NOT_FOUND"


def test_revision_creates_new_draft_on_same_version(plan_id):
    sid = _sheet(plan_id, tension_source=_uniform(500.0, 500.0),
                 loads={"radial_n": 3000.0}, sweep=SWEEP_45,
                 tmin=100.0, tmax=2000.0).json()["load_sheet"]["load_sheet_id"]
    r = client.post(f"/load-sheets/{sid}/revisions", json={
        "loads": {"radial_n": 2000.0},
        "sweep": {"start_deg": 0.0, "end_deg": 180.0, "step_deg": 20.0},
    })
    assert r.status_code == 201, r.text
    rev = r.json()["load_sheet"]
    assert rev["status"] == "draft" and rev["revises"] == sid
    assert rev["plan_ref"]["version"] == 1
    assert rev["case"]["loads"]["radial_n"] == 2000.0
    assert rev["case"]["angles"]["angle_count"] == 10
    # 未提供字段沿用原单
    assert rev["tension_source"]["left_n"] == 500.0
    assert rev["case"]["tension_min_n"] == 100.0
    # 原草稿保留
    assert client.get(f"/load-sheets/{sid}").json()["load_sheet"]["status"] == "draft"


def test_discard_draft(plan_id):
    sid = _sheet(plan_id).json()["load_sheet"]["load_sheet_id"]
    assert client.delete(f"/load-sheets/{sid}").status_code == 204
    assert client.get(f"/load-sheets/{sid}").status_code == 404


def test_list_and_404(plan_id):
    rows = client.get("/load-sheets", params={"plan_id": plan_id}).json()["load_sheets"]
    assert rows
    adopted = [r for r in rows if r["status"] == "adopted"]
    assert adopted
    assert client.get("/load-sheets/lod_missing").status_code == 404
    assert client.post("/plans/whl_missing/load-sheets", json={
        "tension_source": _uniform(), "sweep": SWEEP_30,
        "loads": {"radial_n": 100.0},
        "tension_min_n": 300.0, "tension_max_n": 1500.0}).status_code == 404


def test_version_pin_and_determinism(plan_id):
    # 方案新增版本后，?version=1 的校核单仍绑定 v1，重复计算结果一致
    client.post(f"/plans/{plan_id}/versions", json={**SPEC, "name": "v2"})
    r1 = _sheet(plan_id, version=1, name="a")
    r2 = _sheet(plan_id, version=1, name="b")
    assert r1.status_code == r2.status_code == 201
    d1, d2 = r1.json()["load_sheet"], r2.json()["load_sheet"]
    assert d1["plan_ref"]["version"] == 1
    canon = lambda x: json.dumps(x, sort_keys=True, ensure_ascii=False)
    assert canon(d1["result"]) == canon(d2["result"])
    assert canon(d1["case"]) == canon(d2["case"])
    # 不存在版本
    r = _sheet(plan_id, version=99)
    assert r.status_code == 404 and r.json()["error"]["code"] == "VERSION_NOT_FOUND"


# ---------------------------------------------------------------------------
# 已定稿调校批次作为实测张力来源
# ---------------------------------------------------------------------------

BATCH_SPEC = {
    "name": "b",
    "calibration_curve": [{"reading": 10.0, "tension_n": 400.0},
                          {"reading": 20.0, "tension_n": 900.0},
                          {"reading": 30.0, "tension_n": 1500.0}],
    "radial_zero_mm": 0.0, "lateral_zero_mm": 0.0, "thread_pitch_mm": 0.454,
    "rim_influence_radial_mm_per_turn": 0.08,
    "rim_influence_lateral_mm_per_turn": 0.20,
    "tension_transfer": 0.5, "tension_min_n": 400.0, "tension_max_n": 1400.0,
}


def _finalized_batch(plan_id):
    bid = client.post(f"/plans/{plan_id}/batches?version=1",
                      json=BATCH_SPEC).json()["batch"]["batch_id"]
    holes = [s["rim_hole"] for s in
             client.get(f"/batches/{bid}").json()["batch"]["spokes"]]
    readings = [{"rim_hole": h, "gauge_reading": 20.0,
                 "radial_mm": 0.0, "lateral_mm": 0.0} for h in holes]
    client.post(f"/batches/{bid}/measurements", json={"readings": readings})
    cands = client.get(f"/batches/{bid}/proposals").json()["candidates"]
    noidx = next(i for i, x in enumerate(cands) if x["label"] == "no_action")
    client.post(f"/batches/{bid}/rounds/confirm",
                json={"candidate_index": noidx})
    client.post(f"/batches/{bid}/rounds/complete")
    client.post(f"/batches/{bid}/finalize")
    return bid


def test_finalized_batch_source(sym_plan_id):
    plan_id = sym_plan_id
    bid = _finalized_batch(plan_id)
    r = _sheet(plan_id,
               tension_source={"type": "finalized_batch", "batch_id": bid},
               loads={"radial_n": 1000.0})
    assert r.status_code == 201, r.text
    d = r.json()["load_sheet"]
    assert d["tension_source"]["type"] == "finalized_batch"
    assert d["tension_source"]["batch_id"] == bid
    assert d["tension_source"]["sampled_round"] == 1
    # 末轮逐条实测张力：读数 20 -> 900 N
    assert all(p["pretension_n"] == 900.0 for p in d["pretension"])


def test_non_finalized_batch_rejected(plan_id):
    bid = client.post(f"/plans/{plan_id}/batches?version=1",
                      json=BATCH_SPEC).json()["batch"]["batch_id"]
    r = _sheet(plan_id,
               tension_source={"type": "finalized_batch", "batch_id": bid},
               loads={"radial_n": 1000.0})
    assert r.status_code == 400
    err = r.json()["error"]
    assert err["code"] == "TENSION_SOURCE_INVALID"
    assert err["details"]["status"] == "collecting"


def test_batch_plan_version_mismatch_rejected(plan_id):
    bid = _finalized_batch(plan_id)
    # 批次绑 v1；校核单取 v2 应拒绝
    client.post(f"/plans/{plan_id}/versions", json={**SPEC, "name": "v2b"})
    r = _sheet(plan_id, version=2,
               tension_source={"type": "finalized_batch", "batch_id": bid},
               loads={"radial_n": 1000.0})
    assert r.status_code == 400
    assert r.json()["error"]["code"] == "TENSION_SOURCE_INVALID"


# ---------------------------------------------------------------------------
# 领域单元：模型重建与力学
# ---------------------------------------------------------------------------

def test_build_spoke_model_from_snapshot(plan_id):
    snap = client.get(f"/plans/{plan_id}/versions/1").json()
    model = service_load.build_spoke_model(snap)
    assert len(model["spokes"]) == 36
    for sp in model["spokes"]:
        assert sp["k_n_per_mm"] > 0
        assert len(sp["a"]) == 4 and len(sp["g"]) == 4
        # g = -a
        assert sp["g"] == tuple(-x for x in sp["a"])
        # 方向为单位向量
        assert math.hypot(sp["a"][0], sp["a"][1], sp["a"][2]) > 0


def test_zero_load_prebalanced_has_zero_increment(sym_plan_id):
    # 左右等张力 + 对称轮：外载为零时增量位移为零，张力恒等于预张力
    snap = client.get(f"/plans/{sym_plan_id}/versions/1").json()
    model = service_load.build_spoke_model(snap)
    T0 = [900.0] * 36
    sweep = service_load.run_sweep(
        model, T0, [0.0, 90.0, 180.0],
        {"radial_n": 0.0, "lateral_n": 0.0, "torque_nmm": 0.0}, 1e-9)
    for ang in sweep["angles"]:
        assert ang["hub_displacement_mm"]["x"] == 0.0
        assert ang["hub_displacement_mm"]["y"] == 0.0
    for row in sweep["tensions"]:
        assert row == pytest.approx([900.0] * 36, abs=1e-6)
