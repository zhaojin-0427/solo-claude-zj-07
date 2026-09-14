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
