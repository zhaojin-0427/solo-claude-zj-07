"""Pydantic 请求模型：轮圈、花鼓、每侧穿法与优化输入。"""

from __future__ import annotations

from typing import Literal, Optional

from pydantic import BaseModel, Field, model_validator

Side = Literal["left", "right"]

# 判定两个孔位角度重复的最小间隔（度）
MIN_ANGLE_SEPARATION_DEG = 1e-3


class RimHoleSpec(BaseModel):
    """非等距圈孔表的一条记录。"""

    id: int = Field(ge=0, description="圈孔唯一编号")
    angle_deg: float = Field(ge=0.0, lt=360.0, description="圆周角（度，与标准模式孔号递增同向）")
    side: Side = Field(description="该孔归属侧")
    axial_offset_mm: float = Field(default=0.0, ge=0.0, description="该孔相对轮圈中心线的横向偏移")


def _check_angle_table(items: list[tuple[float, int]], label: str) -> None:
    """角度重复校验：items 为 (角度, 编号)，按角序检查相邻间隔（含跨 0° 的首尾）。"""
    ordered = sorted(items)
    for (a0, i0), (a1, i1) in zip(ordered, ordered[1:]):
        if a1 - a0 < MIN_ANGLE_SEPARATION_DEG:
            raise ValueError(f"{label} {i0} 与 {i1} 的圆周角重复（{a0}° / {a1}°）")
    if len(ordered) > 1:
        a0, i0 = ordered[0]
        a1, i1 = ordered[-1]
        if a0 + 360.0 - a1 < MIN_ANGLE_SEPARATION_DEG:
            raise ValueError(f"{label} {i1} 与 {i0} 的圆周角重复（跨 0°：{a1}° / {a0}°）")


class RimSpec(BaseModel):
    """轮圈规格。

    valve_position: 阀孔位于该编号圈孔与角序下一圈孔之间。
    标准等距模式：圈孔编号 0..holes-1，偶数孔在右侧（驱动侧），奇数孔在左侧。
    hole_table: 非等距钻孔（如成对辐条孔）时逐孔给出唯一编号、圆周角、侧别与
                轴向偏移；提供时覆盖等距模式与 hole_offset_* 字段，条数须等于 holes。
    """

    erd_mm: float = Field(gt=0, description="有效轮圈直径 ERD")
    holes: int = Field(ge=8, le=64, description="圈孔总数")
    hole_offset_left_mm: float = Field(default=0.0, ge=0.0, description="左侧圈孔相对轮圈中心线的横向偏移（等距模式）")
    hole_offset_right_mm: float = Field(default=0.0, ge=0.0, description="右侧圈孔相对轮圈中心线的横向偏移（等距模式）")
    valve_position: int = Field(default=0, ge=0, description="阀孔位于该编号圈孔与角序下一圈孔之间")
    hole_table: Optional[list[RimHoleSpec]] = Field(
        default=None, description="非等距圈孔表；提供时覆盖标准等距模式，条数须等于 holes"
    )

    @model_validator(mode="after")
    def _check_hole_table(self):
        tab = self.hole_table
        if tab is None:
            return self
        ids = [h.id for h in tab]
        dup = sorted({i for i in ids if ids.count(i) > 1})
        if dup:
            raise ValueError(f"圈孔编号重复: {dup}")
        if len(tab) != self.holes:
            raise ValueError(f"孔表覆盖 {len(tab)} 个圈孔，与 holes={self.holes} 不一致")
        _check_angle_table([(h.angle_deg, h.id) for h in tab], "圈孔")
        n_left = sum(1 for h in tab if h.side == "left")
        n_right = len(tab) - n_left
        if n_left != n_right:
            raise ValueError(f"左右侧圈孔数量须相等（左 {n_left} / 右 {n_right}）")
        if n_left % 2 != 0:
            raise ValueError(f"每侧圈孔数须为偶数（每侧 {n_left}）")
        return self


class HubSpec(BaseModel):
    """花鼓规格（左右法兰分别给出）。

    法兰孔角度：默认右法兰 0° 起、左法兰错开半个节距；
    可用 flange_phase_*_deg 设置该侧起始相位，或用 flange_angles_*_deg 逐孔给出角度。
    两者对同一法兰互斥；逐孔角度条数须等于 holes_per_flange。
    """

    holes_per_flange: int = Field(ge=4, le=32, description="每侧法兰孔数")
    flange_pcd_left_mm: float = Field(gt=0, description="左法兰孔节圆直径")
    flange_pcd_right_mm: float = Field(gt=0, description="右法兰孔节圆直径")
    center_to_flange_left_mm: float = Field(gt=0, description="花鼓中心到左法兰距离")
    center_to_flange_right_mm: float = Field(gt=0, description="花鼓中心到右法兰距离")
    spoke_hole_diameter_mm: float = Field(default=2.4, gt=0, description="法兰辐条孔直径")
    flange_phase_left_deg: Optional[float] = Field(default=None, ge=0.0, lt=360.0, description="左法兰孔起始相位（度）")
    flange_phase_right_deg: Optional[float] = Field(default=None, ge=0.0, lt=360.0, description="右法兰孔起始相位（度）")
    flange_angles_left_deg: Optional[list[float]] = Field(default=None, description="左法兰逐孔角度（度）")
    flange_angles_right_deg: Optional[list[float]] = Field(default=None, description="右法兰逐孔角度（度）")

    @model_validator(mode="after")
    def _check_flange_angles(self):
        for side in ("left", "right"):
            phase = getattr(self, f"flange_phase_{side}_deg")
            angles = getattr(self, f"flange_angles_{side}_deg")
            if phase is not None and angles is not None:
                raise ValueError(f"{side} 法兰不能同时设置起始相位与逐孔角度")
            if angles is None:
                continue
            if len(angles) != self.holes_per_flange:
                raise ValueError(
                    f"{side} 法兰逐孔角度覆盖 {len(angles)} 孔，与 holes_per_flange={self.holes_per_flange} 不一致"
                )
            for a in angles:
                if not 0.0 <= a < 360.0:
                    raise ValueError(f"{side} 法兰孔角度 {a} 超出 [0, 360) 范围")
            _check_angle_table([(a, j) for j, a in enumerate(angles)], f"{side} 法兰孔")
        return self


class SideLacing(BaseModel):
    """单侧穿法。

    cross:    交叉数（0 = 径向）
    heads_in: 哪一组辐条头朝法兰内侧（从法兰外侧穿入）：leading（顺向）或 trailing（逆向）
    """

    cross: int = Field(ge=0, le=12)
    heads_in: Literal["leading", "trailing"] = "trailing"


class MappingEntry(BaseModel):
    """自定义孔位映射的一条记录（用于 mapping_override）。"""

    side: Side
    rim_hole: int = Field(ge=0)
    hub_hole: int = Field(ge=0)


class WheelSpec(BaseModel):
    """完整轮组输入。"""

    name: str = "wheel"
    rim: RimSpec
    hub: HubSpec
    left: SideLacing = Field(default_factory=lambda: SideLacing(cross=3))
    right: SideLacing = Field(default_factory=lambda: SideLacing(cross=3))
    spoke_diameter_mm: float = Field(default=2.0, gt=0, description="辐条杆径")
    valve_clearance_min_mm: float = Field(
        default=0.0, ge=0.0, description="阀孔净空下限 mm；自动穿法搜索时须满足，无解返回 LACING_INFEASIBLE"
    )
    mapping_override: Optional[list[MappingEntry]] = Field(
        default=None, description="可选自定义孔位映射；提供时必须覆盖全部圈孔且不得重复"
    )

    @model_validator(mode="after")
    def _check_mapping_override(self):
        mo = self.mapping_override
        if mo is None:
            return self
        seen: dict[int, int] = {}
        for idx, m in enumerate(mo):
            if m.rim_hole in seen:
                raise ValueError(
                    f"圈孔 {m.rim_hole} 被重复占用（第 {seen[m.rim_hole]} 与第 {idx} 条映射），"
                    f"每个圈孔只能穿一根辐条"
                )
            seen[m.rim_hole] = idx
        return self


class InventorySpoke(BaseModel):
    length_mm: float = Field(gt=0)
    count: Optional[int] = Field(default=None, ge=0, description="库存数量；缺省表示不限")


class OptimizeSpec(BaseModel):
    """辐条/垫圈组合优化输入。"""

    inventory: list[InventorySpoke] = Field(min_length=1, description="库存标准辐条长度")
    washers_mm: list[float] = Field(default_factory=list, description="可用垫圈厚度（每个条帽最多一片）")
    tension_min_n: float = Field(default=600.0, ge=0, description="张力下限 N")
    tension_max_n: float = Field(default=1400.0, gt=0, description="张力上限 N")
    length_tolerance_mm: float = Field(default=1.0, ge=0, description="允许长度误差 mm")
    spoke_thread_length_mm: float = Field(default=9.0, gt=0, description="辐条螺纹总长 mm")
    min_thread_engagement_mm: float = Field(default=6.0, ge=0, description="最小螺纹啮合 mm")
    max_protrusion_mm: float = Field(default=1.0, ge=0, description="条帽顶端允许最大外露 mm")
    limit: int = Field(default=10, ge=1, le=100)

    @model_validator(mode="after")
    def _check_tension_window(self):
        if self.tension_max_n <= self.tension_min_n:
            raise ValueError("tension_max_n 必须大于 tension_min_n")
        if self.min_thread_engagement_mm > self.spoke_thread_length_mm:
            raise ValueError("min_thread_engagement_mm 不能大于 spoke_thread_length_mm")
        return self


# ---------------------------------------------------------------------------
# 调校批次
# ---------------------------------------------------------------------------

class CalibrationPoint(BaseModel):
    """张力计校准曲线标定点：读数 -> 实际张力 N（相邻点之间线性插值）。"""

    reading: float = Field(description="张力计读数（表盘分度，按实际仪表单位）")
    tension_n: float = Field(ge=0.0, description="对应的实际张力 N")


class SideTensionLimit(BaseModel):
    min_n: float = Field(ge=0.0)
    max_n: float = Field(gt=0.0)

    @model_validator(mode="after")
    def _check(self):
        if self.max_n <= self.min_n:
            raise ValueError("max_n 必须大于 min_n")
        return self


class TensionLimitOverride(BaseModel):
    """左右侧分别的张力窗口（用于碟形轮两侧不同上限）；缺省侧回落全局值。"""

    left: Optional[SideTensionLimit] = None
    right: Optional[SideTensionLimit] = None


class TuningBatchCreate(BaseModel):
    """创建调校批次：从不可变方案版本取数，本请求只填调校仪器与轮圈影响参数。"""

    name: str = "tuning"
    calibration_curve: list[CalibrationPoint] = Field(min_length=2, description="张力计校准曲线，读数严格递增")
    radial_zero_mm: float = Field(description="百分表径向零位读数 mm（测点径向读数减去该值为相对跳动）")
    lateral_zero_mm: float = Field(description="百分表横向零位读数 mm（正方向取右/驱动侧）")
    thread_pitch_mm: float = Field(gt=0.0, description="辐条螺纹螺距 mm/圈")
    rim_influence_radial_mm_per_turn: float = Field(
        ge=0.0, description="轮圈径向影响系数：一根条帽收紧 1 圈在本孔产生的半径变化 mm（取正值）")
    rim_influence_lateral_mm_per_turn: float = Field(
        ge=0.0, description="轮圈横向影响系数：一根条帽收紧 1 圈在本孔产生的横向偏移 mm（右收紧=向右）")
    tension_transfer: float = Field(
        default=0.5, gt=0.0, le=1.0,
        description="张紧传递系数 τ（0,1]：条帽拉入位移中由辐条弹性承担的比例，"
                    "其余由轮圈弯曲吸收；ΔT=τ·A·E·P/L·u，典型 0.3~0.7")
    tension_min_n: float = Field(default=0.0, ge=0.0, description="全局张力下限 N")
    tension_max_n: float = Field(default=1600.0, gt=0.0, description="全局张力上限 N")
    tension_limits_override: Optional[TensionLimitOverride] = Field(
        default=None, description="左右侧各自的张力窗口；缺省侧使用全局值")
    radial_tolerance_mm: float = Field(default=0.3, ge=0.0, description="径向（去偏心后）跳动超限门限 mm")
    lateral_tolerance_mm: float = Field(default=0.3, ge=0.0, description="横向（去碟形后）跳动超限门限 mm")
    max_turns_per_spoke: float = Field(
        default=2.0, gt=0.0, description="一轮内单根辐条累计转动量上限（圈，按 1/8 取整）")
    step_start_angle_deg: float = Field(
        default=0.0, ge=0.0, lt=360.0, description="步进编排起始圆周角（度，自该角沿角序、左右交替）")

    @model_validator(mode="after")
    def _check_batch(self):
        curve = self.calibration_curve
        for p0, p1 in zip(curve, curve[1:]):
            if p1.reading <= p0.reading:
                raise ValueError(
                    f"校准曲线读数必须严格递增（{p0.reading} 之后出现 {p1.reading}）")
            if p1.tension_n + 1e-9 < p0.tension_n:
                raise ValueError(
                    f"校准曲线张力必须单调不减（读数 {p0.reading}->{p1.reading}）")
        if self.tension_max_n <= self.tension_min_n:
            raise ValueError("tension_max_n 必须大于 tension_min_n")
        if self.tension_limits_override is not None:
            for side, lim in (("left", self.tension_limits_override.left),
                              ("right", self.tension_limits_override.right)):
                if lim is None:
                    continue
        # 1/8 圈整数倍检查
        units = self.max_turns_per_spoke * 8
        if abs(units - round(units)) > 1e-6:
            raise ValueError("max_turns_per_spoke 必须是 1/8 圈的整数倍")
        return self


class SpokeReading(BaseModel):
    """单根辐条测点：张力计读数 + 百分表径向/横向读数（mm，零位在批次上）。"""

    rim_hole: int = Field(ge=0)
    gauge_reading: float
    radial_mm: float
    lateral_mm: float


class MeasurementSubmit(BaseModel):
    readings: list[SpokeReading] = Field(min_length=1, description="沿圈孔角序提交的全部测点")


class LockRequest(BaseModel):
    lock: list[int] = Field(default_factory=list, description="要求锁定（不得调整）的圈孔号")
    unlock: list[int] = Field(default_factory=list, description="解除锁定的圈孔号")

    @model_validator(mode="after")
    def _check(self):
        if not self.lock and not self.unlock:
            raise ValueError("lock 与 unlock 至少提供其一")
        overlap = sorted(set(self.lock) & set(self.unlock))
        if overlap:
            raise ValueError(f"同一孔不能同时锁定与解锁: {overlap}")
        return self


class ConfirmRoundRequest(BaseModel):
    candidate_index: int = Field(default=0, ge=0, description="确认的候选方案编号（0 = 不动作基线）")
