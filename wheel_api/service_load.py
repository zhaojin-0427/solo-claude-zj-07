"""轮组服役载荷校核：刚性花鼓 4 自由度平衡方程、失张退出重解与预张力搜索。

模型（formula_version: wheel-service-load/1.0）：
- 轮圈视为刚性不动体，花鼓为刚体，广义自由度 q = (u_x, u_y, u_z, φ)：
  三个轴向平动 mm 与绕轴（z）转动 rad；
- 辐条只承拉：由不可变方案版本快照中的逐孔长度、截面（πd²/4）、
  法兰位置与入圈角重建三维方向 e_i（毂→圈），轴向刚度 k_i = E·A/L_i；
- 小变形下辐条伸长 ΔL_i = g_i·q，g_i = −a_i，
  a_i = (e_x, e_y, e_z, r_i·(e_i·t_i)) 为辐条张力对花鼓的广义力方向
  （r_i = 该侧法兰半径，t_i = 法兰孔切向）；
- 平衡：K·q = P + Σ_松 T0_i·g_i，K = Σ k_i g_i g_iᵀ（仅受力辐条参与），
  P = 载荷系数 × (F_r·cosθ, F_r·sinθ, F_l, M) 为外载荷广义力；
- 逐角度求解：线性解中 T_i = T0_i + k_i·g_i·q < 0 的辐条判定为失张，
  退出受力（张力置 0、不进刚度矩阵）后重新求解；退出辐条在当前位移下
  自由张力回正到 >0 时重新加入，直到受力集合稳定（互补）；
- 刚度矩阵奇异（如径向穿法承受扭矩）、受力集合不收敛或平衡残差超限时，
  指出对应工况角度；
- 预张力搜索：锁定孔不动，其余孔在允许调节范围（±ΔT、步长、且不越
  服役张力窗口）内做单孔与左右就近配对的贪心下山，按
  违规数 → 包络最大张力 → 改动辐条数 → 总调节量排序。
  无失张时张力增量与预张力无关（线性），用全受力增量矩阵快速评估，
  只有出现失张候选时才跑完整互补重解。

假设静止时全部辐条张紧、轮圈变形已平衡预张力（Σ T0 a = 0），
故全受力时增量方程右侧只有外载荷；有辐条失张后，其失去的预张力
作为平衡力进入右侧（−Σ_松 T0 a = +Σ_松 T0 g）。
"""

from __future__ import annotations

import math

from .errors import WheelError
from .geometry import r3
from .schemas import WheelSpec
from . import geometry

SERVICE_FORMULA_VERSION = "wheel-service-load/1.0"

# 钢辐条弹性模量 N/mm²（与调校模型一致）
STEEL_E_N_PER_MM2 = 205_900.0

# 失张判定 / 重新加入的张力容差 N
SLACK_TOL_N = 1e-6

# 主动集（受力集合）迭代上限：每根辐条平均两次以上仍不稳定即判不收敛
ACTIVE_SET_ITER_CAP = 4

SERVICE_FORMULAS = {
    "dof": "花鼓广义自由度 q = (u_x, u_y, u_z, φ)：三轴平动 mm + 绕轴转动 rad；轮圈刚性固定",
    "spoke_direction": "e_i = (圈孔位置 − 法兰孔位置)/|·|，由逐孔长度、杆径、法兰 PCD/偏距、实际圈/孔角度重建（毂→圈）",
    "axial_stiffness": "k_i = E·A/L_i，A = π·d²/4，E = 205900 N/mm²（钢），L_i = 该孔实际辐条长度",
    "generalized_direction": "a_i = (e_x, e_y, e_z, r_i·(e_i·t_i))；g_i = −a_i，t_i = 法兰孔切向(−sinθ_h, cosθ_h, 0)，r_i = 法兰半径",
    "elongation": "ΔL_i = g_i·q（平动 −e·u；转动 −r_i·(e_i·t_i)·φ）；ΔT_i = k_i·ΔL_i",
    "equilibrium": "Σ_受力 T_i·a_i + P = 0；K·q = P + Σ_失张 T0_i·g_i，K = Σ_受力 k_i·g_i·g_iᵀ",
    "external_load": "P(θ) = 载荷系数 × (F_r·cosθ, F_r·sinθ, F_l, M)；F_r 径向（θ=孔角方向），F_l 侧向（正=右/驱动侧），M 绕轴（正=驱动，N·mm）",
    "slack_release": "T_i ≤ 0 的辐条失张：张力置 0 并退出刚度矩阵后重解；自由张力回正 >0 时重新加入，直到受力集合稳定",
    "residual": "相对平衡残差：|Σ T_i a_i + P| 的力分量 / ΣT、力矩分量 / (R·ΣT)，任一超门限即判工况无解",
    "envelope": "逐角度张力包络：每根辐条所有角度下的最小/最大张力及发生角度",
    "pretension_search": "锁定孔不动；单孔 ±步长 与左右就近配对 ±步长 贪心下山；结果预张力须在服役张力窗口与 ±最大调节量内",
    "ranking": "预张力方案排序：违规（失张/低于下限/超过上限的孔-角度对）数↑ → 全角度最大张力↑ → 改动辐条数↑ → 总调节量↑",
}


def r6(x: float) -> float:
    return round(float(x), 6)


# ---------------------------------------------------------------------------
# 由不可变方案快照重建逐孔力学模型
# ---------------------------------------------------------------------------

def build_spoke_model(plan_snapshot: dict) -> dict:
    """从方案版本快照重建逐孔方向、法兰切向、轴向刚度（不重新搜索穿法）。"""
    spec = WheelSpec.model_validate(plan_snapshot["inputs"])
    layout = geometry.resolve_layout(spec)
    geo = plan_snapshot["geometry"]
    rim_r = float(geo["rim_radius_mm"])
    dia = float(geo["spoke_diameter_mm"])
    area = math.pi * dia * dia / 4.0

    hub_index = {
        (e["side"], e["rim_hole"]): e["hub_hole"]
        for e in plan_snapshot["lacing"]["mapping"]
    }
    flange_r = {
        "left": spec.hub.flange_pcd_left_mm / 2.0,
        "right": spec.hub.flange_pcd_right_mm / 2.0,
    }
    flange_w = {
        "left": spec.hub.center_to_flange_left_mm,
        "right": spec.hub.center_to_flange_right_mm,
    }

    spokes = []
    for h in sorted(geo["per_hole"], key=lambda x: x["rim_hole"]):
        hole = h["rim_hole"]
        side = h["side"]
        a_r = math.radians(h["rim_angle_deg"])
        j = hub_index[(side, hole)]
        a_h = layout.hub_angles[side][j]
        offset = layout.rim_offsets[hole]
        r_f = flange_r[side]
        w_f = flange_w[side]
        z_rim = offset if side == "right" else -offset
        z_hub = w_f if side == "right" else -w_f
        rim_pos = (rim_r * math.cos(a_r), rim_r * math.sin(a_r), z_rim)
        hub_pos = (r_f * math.cos(a_h), r_f * math.sin(a_h), z_hub)
        qx, qy, qz = (rim_pos[0] - hub_pos[0],
                      rim_pos[1] - hub_pos[1],
                      rim_pos[2] - hub_pos[2])
        qnorm = math.sqrt(qx * qx + qy * qy + qz * qz)
        ex, ey, ez = qx / qnorm, qy / qnorm, qz / qnorm
        tx, ty = -math.sin(a_h), math.cos(a_h)
        e_tangent = ex * tx + ey * ty
        length = float(h["length_mm"])
        k = STEEL_E_N_PER_MM2 * area / length
        # 广义力方向 a = (e_x, e_y, e_z, r·(e·t))；g = −a
        a4 = (ex, ey, ez, r_f * e_tangent)
        spokes.append({
            "rim_hole": hole,
            "side": side,
            "rim_angle_rad": a_r,
            "length_mm": length,
            "k_n_per_mm": k,
            "a": a4,
            "g": (-a4[0], -a4[1], -a4[2], -a4[3]),
        })
    return {
        "rim_radius_mm": rim_r,
        "spoke_diameter_mm": dia,
        "spokes": spokes,
        "formula_version": plan_snapshot.get("formula_version"),
    }


# ---------------------------------------------------------------------------
# 4x4 线性方程（部分选主元高斯消去；零主元识别奇异/零载荷自由度）
# ---------------------------------------------------------------------------

def _solve4(A: list[list[float]], b: list[float], scale: float):
    """解对称正定/半正定系统 K q = b（部分选主元高斯消去）。

    K 为受力辐条刚度和，对称半正定；奇异只可能发生在某个自由度完全无
    刚度（如径向穿法的绕轴转动）。选主元在剩余子矩阵全范围进行，行、
    列同步交换以保持对称，列置换通过 perm 还回原变量顺序。

    返回 (q, singular_nullspace_load)：零主元行右端也为零时自由变量取 0；
    零空间上有载（如径向穿法承受扭矩）时返回 singular=True。
    """
    M = [A[i][:] + [b[i]] for i in range(4)]
    pivot_scale = max(scale, 1e-30)
    perm = list(range(4))   # 当前列 -> 原变量列
    qperm = [0.0] * 4       # 当前（置换后）变量序的解
    pivot_row = 0
    for col in range(4):
        # 在剩余行/列子块中找最大主元（K 对称：行列同步交换）
        best_r, best_c, best_v = pivot_row, col, 0.0
        for rr in range(pivot_row, 4):
            for cc in range(col, 4):
                if abs(M[rr][cc]) > abs(best_v):
                    best_r, best_c, best_v = rr, cc, M[rr][cc]
        if abs(best_v) < 1e-12 * pivot_scale:
            break  # 剩余子块全零：pivot_row..3 行为零空间约束
        if best_r != pivot_row:
            M[pivot_row], M[best_r] = M[best_r], M[pivot_row]
        if best_c != col:
            for rr in range(4):
                M[rr][col], M[rr][best_c] = M[rr][best_c], M[rr][col]
            perm[col], perm[best_c] = perm[best_c], perm[col]
        pivot = M[pivot_row][col]
        for rr in range(pivot_row + 1, 4):
            factor = M[rr][col] / pivot
            if factor == 0.0:
                continue
            for cc in range(col, 5):
                M[rr][cc] -= factor * M[pivot_row][cc]
        pivot_row += 1
    # 零空间行（pivot_row..3）：右端必须也为零，否则该载荷无刚度承载
    if any(abs(M[rr][4]) > 1e-8 * pivot_scale for rr in range(pivot_row, 4)):
        return None, True
    # 回代（置换序）；自由变量（与 pivot_row..3 对应的列）保持 0
    for row in range(pivot_row - 1, -1, -1):
        s = M[row][4]
        for cc in range(row + 1, 4):
            s -= M[row][cc] * qperm[cc]
        qperm[row] = s / M[row][row]
    q = [0.0] * 4
    for k in range(4):
        q[perm[k]] = qperm[k]
    return q, False


class _ActiveSetSolver:
    """单角度主动集求解；主动集合 -> 4x4 因子缓存（同一轮扫描重复利用）。"""

    def __init__(self, model: dict):
        self.spokes = model["spokes"]
        self.n = len(self.spokes)
        self.rim_r = model["rim_radius_mm"]
        self._cache: dict[tuple[bool, ...], tuple] = {}

    def _assemble(self, active: list[bool]):
        K = [[0.0] * 4 for _ in range(4)]
        scale = 0.0
        for i, sp in enumerate(self.spokes):
            if not active[i]:
                continue
            g = sp["g"]
            k = sp["k_n_per_mm"]
            scale = max(scale, k * max(abs(v) for v in g) ** 2)
            for p in range(4):
                kp = k * g[p]
                Kp = K[p]
                for qcol in range(4):
                    Kp[qcol] += kp * g[qcol]
        return K, max(scale, 1e-30)

    def solve(self, T0: list[float], P: tuple[float, float, float, float]):
        """返回 (q, tensions, active_mask, residual) 或 None（奇异/不收敛）。

        不假设预张力自平衡：S0 = Σ T0_i·a_i 为预张力广义合力（实测张力
        通常带有残余碟形/扭矩失衡），与外载荷一同由花鼓位移平衡。
        """
        active = [True] * self.n
        seen: set[tuple[bool, ...]] = set()
        q = [0.0] * 4
        tensions = list(T0)
        s0 = [0.0] * 4
        for j, sp in enumerate(self.spokes):
            a = sp["a"]
            t0 = T0[j]
            for p in range(4):
                s0[p] += t0 * a[p]
        for _iteration in range(self.n * ACTIVE_SET_ITER_CAP + 8):
            mask = tuple(active)
            if mask in seen:
                return None  # 主动集循环振荡，不收敛
            seen.add(mask)
            cached = self._cache.get(mask)
            if cached is None:
                K, scale = self._assemble(active)
                cached = (K, scale)
                self._cache[mask] = cached
            K, scale = cached
            # K q = P + S0 + Σ_失张 T0 g
            # （全受力时 K q = P + S0；失张后该辐条的预张力项 T0 a
            # 从 S0 中消失，而其平衡贡献变为 −T0 g，等价于 +T0 g）
            rhs = [P[p] + s0[p] for p in range(4)]
            for j, sp in enumerate(self.spokes):
                if not active[j]:
                    gj = sp["g"]
                    for p in range(4):
                        rhs[p] += T0[j] * gj[p]
            q, singular = _solve4(K, rhs, scale)
            if singular:
                return None
            new_active = active[:]
            for i, sp in enumerate(self.spokes):
                gi = sp["g"]
                free_t = T0[i] + sp["k_n_per_mm"] * (
                    gi[0] * q[0] + gi[1] * q[1] + gi[2] * q[2] + gi[3] * q[3])
                if active[i]:
                    if free_t < -SLACK_TOL_N:
                        new_active[i] = False
                else:
                    if free_t > SLACK_TOL_N:
                        new_active[i] = True
            if new_active == active:
                tensions = [0.0] * self.n
                for i, sp in enumerate(self.spokes):
                    gi = sp["g"]
                    if active[i]:
                        tensions[i] = max(0.0, T0[i] + sp["k_n_per_mm"] * (
                            gi[0] * q[0] + gi[1] * q[1]
                            + gi[2] * q[2] + gi[3] * q[3]))
                residual = self._residual(active, tensions, P)
                return q, tensions, active, residual
            active = new_active
        return None  # 迭代上限

    def _residual(self, active, tensions, P):
        fx = fy = fz = mz = 0.0
        total_t = 0.0
        for i, sp in enumerate(self.spokes):
            if not active[i]:
                continue
            t = tensions[i]
            a = sp["a"]
            fx += t * a[0]
            fy += t * a[1]
            fz += t * a[2]
            mz += t * a[3]
            total_t += t
        force_n = math.hypot(math.hypot(fx + P[0], fy + P[1]), fz + P[2])
        moment_nmm = abs(mz + P[3])
        rel_f = force_n / max(1.0, total_t)
        rel_m = moment_nmm / max(1.0, self.rim_r * total_t)
        return {"force_rel": rel_f, "moment_rel": rel_m,
                "force_n": force_n, "moment_nmm": moment_nmm}


# ---------------------------------------------------------------------------
# 工况扫描
# ---------------------------------------------------------------------------

def _infeasible(angle_deg: float, reason: str, detail: dict):
    raise WheelError(
        "LOAD_CASE_INFEASIBLE",
        f"角度 {r3(angle_deg)}° 工况下平衡方程{reason}",
        {"angle_deg": r3(angle_deg), "reason": reason, **detail},
    )


def run_sweep(model: dict, T0: list[float], angles_deg: list[float],
              loads: dict, residual_tol: float, solver: "_ActiveSetSolver | None" = None) -> dict:
    """逐角度求花鼓位移与每根辐条张力（含失张退出重解）。

    loads: 已乘载荷系数的 {radial_n, lateral_n, torque_nmm}。
    返回每角度 q、张力行（孔序）、失张孔、残差；任一角度失败即抛出。
    solver 可复用：主动集合 -> K 矩阵缓存只依赖几何，与预张力无关。
    """
    if solver is None:
        solver = _ActiveSetSolver(model)
    spokes = model["spokes"]
    Fr, Fl, Mm = loads["radial_n"], loads["lateral_n"], loads["torque_nmm"]
    per_angle = []
    tension_rows = []
    active_rows = []
    for angle in angles_deg:
        th = math.radians(angle % 360.0)
        P = (Fr * math.cos(th), Fr * math.sin(th), Fl, Mm)
        out = solver.solve(T0, P)
        if out is None:
            _infeasible(angle, "无解（刚度矩阵奇异或失张受力集合不收敛）",
                        {"active_spoke_count": "—",
                         "loads_n": {"radial": r3(Fr), "lateral": r3(Fl)},
                         "torque_nmm": r3(Mm)})
        q, tensions, active, residual = out
        if residual["force_rel"] > residual_tol or residual["moment_rel"] > residual_tol:
            _infeasible(angle, "平衡残差超限",
                        {"residual_force_rel": r6(residual["force_rel"]),
                         "residual_moment_rel": r6(residual["moment_rel"]),
                         "residual_force_n": r3(residual["force_n"]),
                         "residual_moment_nmm": r3(residual["moment_nmm"]),
                         "tolerance": residual_tol,
                         "active_spoke_count": sum(active)})
        slack = [spokes[i]["rim_hole"] for i in range(len(spokes)) if not active[i]]
        per_angle.append({
            "angle_deg": r3(angle),
            "hub_displacement_mm": {"x": r3(q[0]), "y": r3(q[1]), "z": r3(q[2])},
            "hub_rotation_deg": r3(math.degrees(q[3])),
            "slack_rim_holes": slack,
            "residual": {"force_rel": r6(residual["force_rel"]),
                         "moment_rel": r6(residual["moment_rel"]),
                         "force_n": r3(residual["force_n"]),
                         "moment_nmm": r3(residual["moment_nmm"])},
        })
        tension_rows.append(tensions)
        active_rows.append(active)
    return {"angles": per_angle, "tensions": tension_rows, "active": active_rows}


def linear_delta_matrix(model: dict, angles_deg: list[float],
                        loads: dict) -> list[list[float]] | None:
    """全受力线性解中**仅外载荷**引起的逐孔逐角度张力增量（与预张力无关）。

    用于预张力搜索的快速评估；全受力矩阵奇异时返回 None。
    """
    spokes = model["spokes"]
    n = len(spokes)
    K = [[0.0] * 4 for _ in range(4)]
    for sp in spokes:
        g = sp["g"]
        k = sp["k_n_per_mm"]
        for p in range(4):
            kp = k * g[p]
            for c in range(4):
                K[p][c] += kp * g[c]
    scale = max(abs(K[p][p]) for p in range(4))
    Fr, Fl, Mm = loads["radial_n"], loads["lateral_n"], loads["torque_nmm"]
    deltas = [[] for _ in range(n)]
    for angle in angles_deg:
        th = math.radians(angle % 360.0)
        P = (Fr * math.cos(th), Fr * math.sin(th), Fl, Mm)
        q, singular = _solve4(K, list(P), scale)
        if singular:
            return None
        for i, sp in enumerate(spokes):
            g = sp["g"]
            deltas[i].append(sp["k_n_per_mm"] * (
                g[0] * q[0] + g[1] * q[1] + g[2] * q[2] + g[3] * q[3]))
    return deltas


# ---------------------------------------------------------------------------
# 结果统计：包络、首根松弛/超载、最小余量、最危险角度
# ---------------------------------------------------------------------------

_SEVERITY = {"slack": 0, "over_max": 1, "below_min": 2}


def _spoke_events(tensions, active, tmin, tmax):
    """返回 (每角度事件, 每孔每角度事件类型或 None)。"""
    nang = len(tensions)
    n = len(tensions[0])
    events_by_angle = []
    type_grid = [[None] * n for _ in range(nang)]
    for k in range(nang):
        evs = []
        for i in range(n):
            kind = None
            if not active[k][i]:
                kind = "slack"
            elif tensions[k][i] > tmax[i] + 1e-9:
                kind = "over_max"
            elif tensions[k][i] < tmin[i] - 1e-9:
                kind = "below_min"
            if kind:
                type_grid[k][i] = kind
                evs.append((i, kind))
        events_by_angle.append(evs)
    return events_by_angle, type_grid


def _envelope(model, angles_deg, tensions, tmin, tmax) -> dict:
    spokes = model["spokes"]
    n = len(spokes)
    per_spoke = []
    gmin, gmax = math.inf, -math.inf
    gmin_at = gmax_at = None
    for i, sp in enumerate(spokes):
        vals = [tensions[k][i] for k in range(len(angles_deg))]
        vmin, imin = min((v, k) for k, v in enumerate(vals))
        vmax, imax = max((v, k) for k, v in enumerate(vals))
        if vmin < gmin:
            gmin, gmin_at = vmin, (sp["rim_hole"], angles_deg[imin])
        if vmax > gmax:
            gmax, gmax_at = vmax, (sp["rim_hole"], angles_deg[imax])
        per_spoke.append({
            "rim_hole": sp["rim_hole"],
            "side": sp["side"],
            "min_n": r3(vmin),
            "min_at_angle_deg": r3(angles_deg[imin]),
            "max_n": r3(vmax),
            "max_at_angle_deg": r3(angles_deg[imax]),
            "margin_to_min_n": r3(vmin - tmin[i]),
            "margin_to_max_n": r3(tmax[i] - vmax),
        })
    return {
        "global": {
            "min_n": r3(gmin), "min_rim_hole": gmin_at[0],
            "min_at_angle_deg": r3(gmin_at[1]),
            "max_n": r3(gmax), "max_rim_hole": gmax_at[0],
            "max_at_angle_deg": r3(gmax_at[1]),
        },
        "per_spoke": per_spoke,
    }


def _first_event(model, angles_deg, events_by_angle):
    """沿角度扫描方向：首个出现事件的角度上，按 失张 > 超载 > 低于下限 取最严重者。"""
    spokes = model["spokes"]
    for k, evs in enumerate(events_by_angle):
        if not evs:
            continue
        i, kind = min(evs, key=lambda ie: (_SEVERITY[ie[1]], spokes[ie[0]]["rim_hole"]))
        return {
            "angle_deg": r3(angles_deg[k]),
            "rim_hole": spokes[i]["rim_hole"],
            "side": spokes[i]["side"],
            "type": kind,
        }
    return None


def _minimum_margin(model, angles_deg, tensions, tmin, tmax) -> dict:
    """全孔-全角度上对张力窗口的最小余量（负值=已越限）。"""
    spokes = model["spokes"]
    best = None
    for k in range(len(angles_deg)):
        for i, sp in enumerate(spokes):
            t = tensions[k][i]
            for kind, margin, limit in (
                ("to_min", t - tmin[i], tmin[i]),
                ("to_max", tmax[i] - t, tmax[i]),
            ):
                if best is None or margin < best["margin_n"]:
                    best = {"kind": kind, "margin_n": margin,
                            "rim_hole": sp["rim_hole"], "side": sp["side"],
                            "angle_deg": angles_deg[k],
                            "tension_n": t, "limit_n": limit}
    return {
        "kind": best["kind"],
        "margin_n": r3(best["margin_n"]),
        "rim_hole": best["rim_hole"],
        "side": best["side"],
        "angle_deg": r3(best["angle_deg"]),
        "tension_n": r3(best["tension_n"]),
        "limit_n": r3(best["limit_n"]),
    }


def _worst_angle(model, angles_deg, events_by_angle, tensions, tmin, tmax) -> dict:
    """违规孔数最多、并列时越限量最大的角度。"""
    best = None
    for k, evs in enumerate(events_by_angle):
        excess = 0.0
        for i, kind in evs:
            t = tensions[k][i]
            if kind == "over_max":
                excess = max(excess, t - tmax[i])
            else:
                excess = max(excess, tmin[i] - t)
        key = (len(evs), excess, -k)
        if best is None or key > best[0]:
            best = (key, k)
    k = best[1]
    return {
        "angle_deg": r3(angles_deg[k]),
        "violating_spoke_count": len(events_by_angle[k]),
        "max_excess_n": r3(best[0][1]),
        "slack_rim_holes": [model["spokes"][i]["rim_hole"]
                            for i, kind in events_by_angle[k] if kind == "slack"],
        "rim_holes": [
            {"rim_hole": model["spokes"][i]["rim_hole"],
             "side": model["spokes"][i]["side"], "type": kind,
             "tension_n": r3(0.0 if kind == "slack" else tensions[k][i])}
            for i, kind in sorted(events_by_angle[k],
                                  key=lambda ie: model["spokes"][ie[0]]["rim_hole"])
        ],
    }


def summarize_sweep(model, angles_deg, tensions, active, tmin, tmax) -> dict:
    """统计：包络、首根松弛/超载、最小余量、最危险角度（不含逐角度详情）。"""
    events_by_angle, _ = _spoke_events(tensions, active, tmin, tmax)
    total_violations = sum(len(evs) for evs in events_by_angle)
    slack_pairs = sum(1 for k in range(len(angles_deg))
                      for i in range(len(model["spokes"])) if not active[k][i])
    return {
        "violation_count": total_violations,
        "slack_spoke_angle_pairs": slack_pairs,
        "envelope": _envelope(model, angles_deg, tensions, tmin, tmax),
        "first_event": _first_event(model, angles_deg, events_by_angle),
        "minimum_margin": _minimum_margin(model, angles_deg, tensions, tmin, tmax),
        "worst_angle": _worst_angle(model, angles_deg, events_by_angle,
                                    tensions, tmin, tmax),
        "events_by_angle": events_by_angle,
    }


def _angle_details(model, angles_deg, tensions, active, events_by_angle,
                   hub_rows: list[dict]) -> list[dict]:
    spokes = model["spokes"]
    rows = []
    for k, hub in enumerate(hub_rows):
        violations = [
            {"rim_hole": spokes[i]["rim_hole"], "side": spokes[i]["side"],
             "type": kind,
             "tension_n": r3(0.0 if kind == "slack" else tensions[k][i])}
            for i, kind in sorted(events_by_angle[k],
                                  key=lambda ie: spokes[ie[0]]["rim_hole"])
        ]
        rows.append({
            "angle_deg": r3(angles_deg[k]),
            "hub_displacement_mm": hub["hub_displacement_mm"],
            "hub_rotation_deg": hub["hub_rotation_deg"],
            "residual": hub["residual"],
            "slack_rim_holes": hub["slack_rim_holes"],
            "violations": violations,
            "spoke_tensions_n": [
                {"rim_hole": spokes[i]["rim_hole"], "side": spokes[i]["side"],
                 "active": active[k][i], "tension_n": r3(tensions[k][i])}
                for i in range(len(spokes))
            ],
        })
    return rows


# ---------------------------------------------------------------------------
# 预张力方案搜索
# ---------------------------------------------------------------------------

class _LinearContext:
    """全受力线性评估上下文：因子只依赖几何，q0 随候选预张力重算。"""

    def __init__(self, model: dict):
        spokes = model["spokes"]
        self.model = model
        self.K = [[0.0] * 4 for _ in range(4)]
        for sp in spokes:
            g = sp["g"]
            k = sp["k_n_per_mm"]
            for p in range(4):
                kp = k * g[p]
                for c in range(4):
                    self.K[p][c] += kp * g[c]
        self.scale = max(abs(self.K[p][p]) for p in range(4))

    def preload_response(self, T0) -> list[float] | None:
        """S0/K 引起的逐孔张力变化（与角度无关）；奇异返回 None。"""
        spokes = self.model["spokes"]
        s0 = [0.0] * 4
        for j, sp in enumerate(spokes):
            a = sp["a"]
            for p in range(4):
                s0[p] += T0[j] * a[p]
        q0, singular = _solve4(self.K, s0, self.scale)
        if singular:
            return None
        out = []
        for sp in spokes:
            g = sp["g"]
            out.append(sp["k_n_per_mm"] * (
                g[0] * q0[0] + g[1] * q0[1] + g[2] * q0[2] + g[3] * q0[3]))
        return out


def _evaluate_tensions(model, T0, ctx: "_LinearContext | None", deltas,
                       angles_deg, loads, residual_tol, solver) -> tuple:
    """候选预张力下的张力矩阵：无失张走线性快速路径，否则完整互补扫描。

    deltas 为仅外载荷增量（逐孔逐角度，与预张力无关）；ctx 提供候选预
    张力失衡项 q0 的快速解。ctx/deltas 为 None（全受力矩阵奇异，如径向
    穿法受扭矩）时所有候选走完整主动集扫描。扫描失败的候选在搜索中
    跳过（基线失败在创建校核单时直接报错）。
    """
    n = len(T0)
    if ctx is not None:
        preload_delta = ctx.preload_response(T0)
        if preload_delta is not None:
            fast_ok = True
            tensions = [[0.0] * n for _ in angles_deg]
            for k, _angle in enumerate(angles_deg):
                for i in range(n):
                    t = T0[i] + preload_delta[i] + deltas[i][k]
                    if t <= SLACK_TOL_N:
                        fast_ok = False
                    tensions[k][i] = t
            if fast_ok:
                return tensions, [[True] * n for _ in angles_deg], True
    try:
        sweep = run_sweep(model, T0, angles_deg, loads, residual_tol, solver)
    except WheelError:
        return None, None, False
    return sweep["tensions"], sweep["active"], True


def _candidate_key(T0_adj, base_T0, tensions, active, tmin, tmax) -> tuple:
    n = len(T0_adj)
    violations = 0
    tmax_seen = -math.inf
    for k in range(len(tensions)):
        for i in range(n):
            t = tensions[k][i]
            tmax_seen = max(tmax_seen, t)
            if not active[k][i]:
                violations += 1  # 失张：松条
            elif t > tmax[i] + 1e-9 or t < tmin[i] - 1e-9:
                violations += 1
    changed = sum(1 for i in range(n) if abs(T0_adj[i] - base_T0[i]) > 1e-9)
    total = r3(sum(abs(T0_adj[i] - base_T0[i]) for i in range(n)))
    tie = tuple(r3(v) for v in T0_adj)
    return (violations, r3(tmax_seen), changed, total, tie)


def search_pretension(model, base_T0, tmin, tmax, angles_deg, loads,
                      residual_tol, locked: set[int], max_adjustment_n: float,
                      step_n: float, limit: int, deltas) -> list[dict]:
    """贪心搜索预张力方案（含不调节基线），按官方四项键排序返回。

    deltas 为全受力线性外载荷增量矩阵；None 表示全受力矩阵奇异（如径向
    穿法承受扭矩），全部候选改用完整主动集扫描评估。
    """
    spokes = model["spokes"]
    n = len(spokes)
    ctx = _LinearContext(model) if deltas is not None else None
    solver = _ActiveSetSolver(model)
    hole_idx = {sp["rim_hole"]: i for i, sp in enumerate(spokes)}
    locked_idx = {hole_idx[h] for h in locked}
    adjustable = [i for i in range(n) if i not in locked_idx]

    # 左右就近配对（角距最小的另一侧孔）
    pairs = set()
    for i in adjustable:
        j = min(
            (x for x in adjustable if spokes[x]["side"] != spokes[i]["side"]),
            key=lambda x: abs(math.atan2(
                math.sin(spokes[x]["rim_angle_rad"] - spokes[i]["rim_angle_rad"]),
                math.cos(spokes[x]["rim_angle_rad"] - spokes[i]["rim_angle_rad"]))),
            default=None,
        )
        if j is not None:
            pairs.add((min(i, j), max(i, j)))

    def within(i, value):
        d = value - base_T0[i]
        return (abs(d) <= max_adjustment_n + 1e-9
                and tmin[i] - 1e-9 <= value <= tmax[i] + 1e-9)

    def moves(d):
        """单孔、左右就近配对，以及整侧/整轮协调动作。

        单孔与配对处理局部问题；载荷侧整圈失张必须靠整体抬高预张力
        （单侧抬高会引入碟形失衡，故同时提供两侧同步抬高），这类
        多孔协调动作在统一四项键下与其他动作竞争择优。
        """
        out = []
        for i in adjustable:
            for s in (step_n, -step_n):
                v = d[i] + s
                if within(i, v):
                    dd = d[:]
                    dd[i] = v
                    out.append(dd)
        for i, j in pairs:
            for s in (step_n, -step_n):
                if within(i, d[i] + s) and within(j, d[j] - s):
                    dd = d[:]
                    dd[i] += s
                    dd[j] -= s
                    out.append(dd)

        def group_step(idxs, s):
            if not idxs:
                return None
            dd = d[:]
            for i in idxs:
                if not within(i, dd[i] + s):
                    return None
            for i in idxs:
                dd[i] += s
            return dd

        by_side = {"left": [i for i in adjustable if spokes[i]["side"] == "left"],
                   "right": [i for i in adjustable if spokes[i]["side"] == "right"]}
        # 整侧抬高/降低
        for idxs in (by_side["left"], by_side["right"]):
            for s in (step_n, -step_n):
                dd = group_step(idxs, s)
                if dd is not None:
                    out.append(dd)
        # 两侧同步抬高/降低（保持左右平衡，整体改变预张力水平）
        for s in (step_n, -step_n):
            dd = group_step(adjustable, s)
            if dd is not None:
                out.append(dd)
        return out

    def key_of(d):
        tensions, active, ok = _evaluate_tensions(
            model, d, ctx, deltas, angles_deg, loads, residual_tol, solver)
        if not ok:
            return None
        return _candidate_key(d, base_T0, tensions, active, tmin, tmax), tensions, active

    current = [float(v) for v in base_T0]
    base_ev = key_of(current)
    if base_ev is None:
        # 基线在完整主动集扫描下都失败：创建校核单时已先行报错，这里不应到达
        raise WheelError(
            "LOAD_CASE_INFEASIBLE",
            "基线预张力下平衡方程无解，无法搜索预张力方案", {})
    base_key = base_ev[0]
    # 立即固定不调节基线：贪心迭代会改写 current，必须用独立副本
    accepted = [([float(v) for v in base_T0], base_key)]
    best_key = base_key
    # 每轮扫描全部单孔/配对动作，取严格变优者提交；无改进或达迭代上限停止
    for _ in range(max(24, n * 2)):
        chosen = None
        for dd in moves(current):
            ev = key_of(dd)
            if ev is None:
                continue
            key, _, _ = ev
            if key < best_key:
                chosen = (key, dd)
                best_key = key
        if chosen is None:
            break
        current = chosen[1]
        accepted.append(([float(v) for v in current], chosen[0]))

    # 去重、按四项键排序、打包
    seen = {}
    for d, key in accepted:
        tk = tuple(r3(v) for v in d)
        if tk not in seen or key < seen[tk][1]:
            seen[tk] = (d, key)
    # 打包：按四项键排序；无改动者恒为 baseline（与排序名次无关），
    # 有改动方案按排序名次编号 option_0、option_1…
    ordered = sorted(seen.values(), key=lambda x: x[1])
    packaged = []
    option_no = 0
    for d, key in ordered:
        tensions, active, _ = _evaluate_tensions(
            model, d, ctx, deltas, angles_deg, loads, residual_tol, solver)
        env = _envelope(model, angles_deg, tensions, tmin, tmax)
        adjustments = []
        for i, sp in enumerate(spokes):
            delta = r3(d[i] - base_T0[i])
            if delta != 0.0:
                adjustments.append({
                    "rim_hole": sp["rim_hole"], "side": sp["side"],
                    "from_n": r3(base_T0[i]), "to_n": r3(d[i]), "delta_n": delta,
                })
        label = "baseline" if not adjustments else f"option_{option_no}"
        if adjustments:
            option_no += 1
        packaged.append({
            "label": label,
            "feasible": key[0] == 0,
            "violation_count": key[0],
            "max_tension_n": key[1],
            "changed_spoke_count": key[2],
            "total_adjustment_n": key[3],
            "envelope_global": env["global"],
            "adjustments": sorted(adjustments, key=lambda a: a["rim_hole"]),
            "pretension_n": [
                {"rim_hole": spokes[i]["rim_hole"], "side": spokes[i]["side"],
                 "pretension_n": r3(d[i])}
                for i in range(n)
            ],
        })
        if len(packaged) >= limit:
            break
    return packaged


# ---------------------------------------------------------------------------
# 校核单装配
# ---------------------------------------------------------------------------

def build_check_result(model: dict, pretension: list[float], tmin, tmax,
                       angles_deg: list[float], factored_loads: dict,
                       residual_tol: float, search: dict | None = None) -> dict:
    """完整校核：基线扫描 + 统计 + 可选预张力搜索。"""
    sweep = run_sweep(model, pretension, angles_deg, factored_loads, residual_tol)
    tensions = sweep["tensions"]
    active = sweep["active"]

    summary = summarize_sweep(model, angles_deg, tensions, active, tmin, tmax)
    per_angle = _angle_details(model, angles_deg, tensions, active,
                               summary["events_by_angle"], sweep["angles"])
    result = {
        "angles_deg": [r3(a) for a in angles_deg],
        "violation_count": summary["violation_count"],
        "slack_spoke_angle_pairs": summary["slack_spoke_angle_pairs"],
        "tension_envelope": summary["envelope"],
        "first_event": summary["first_event"],
        "minimum_margin": summary["minimum_margin"],
        "worst_angle": summary["worst_angle"],
        "per_angle": per_angle,
    }
    if search is not None:
        deltas = linear_delta_matrix(model, angles_deg, factored_loads)
        # deltas 为 None（全受力矩阵奇异）：搜索中全部候选走完整主动集扫描
        options = search_pretension(
            model, pretension, tmin, tmax, angles_deg, factored_loads,
            residual_tol, search["locked"], search["max_adjustment_n"],
            search["step_n"], search["limit"], deltas)
        selected = search.get("selected_index", 0)
        if selected is None:
            selected = 0
        if selected >= len(options):
            raise WheelError(
                "PRETENSION_OPTION_NOT_FOUND",
                f"选定的预张力方案编号 {selected} 不存在（共 {len(options)} 个）",
                {"selected_index": selected, "option_count": len(options)})
        result["pretension_search"] = {
            "locked_rim_holes": sorted(search["locked"]),
            "max_adjustment_n": search["max_adjustment_n"],
            "step_n": search["step_n"],
            "options": options,
            "selected_index": selected,
            "selected": options[selected],
            "note": "方案仅为预张力建议，不写回来源调校批次；采用后随校核单一并冻结",
        }
    return result
