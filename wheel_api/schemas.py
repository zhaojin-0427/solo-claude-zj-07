"""Pydantic 请求模型：轮圈、花鼓、每侧穿法与优化输入。"""

from __future__ import annotations

from typing import Literal, Optional

from pydantic import BaseModel, Field, model_validator

Side = Literal["left", "right"]


class RimSpec(BaseModel):
    """轮圈规格。

    valve_position: 阀孔位于第 valve_position 个圈孔与下一个（顺时针，从驱动侧看）圈孔之间。
    圈孔编号 0..holes-1，偶数孔在右侧（驱动侧），奇数孔在左侧。
    """

    erd_mm: float = Field(gt=0, description="有效轮圈直径 ERD")
    holes: int = Field(ge=8, le=64, description="圈孔总数")
    hole_offset_left_mm: float = Field(default=0.0, ge=0.0, description="左侧圈孔相对轮圈中心线的横向偏移")
    hole_offset_right_mm: float = Field(default=0.0, ge=0.0, description="右侧圈孔相对轮圈中心线的横向偏移")
    valve_position: int = Field(default=0, ge=0, description="阀孔位于该编号圈孔与下一圈孔之间")


class HubSpec(BaseModel):
    """花鼓规格（左右法兰分别给出）。"""

    holes_per_flange: int = Field(ge=4, le=32, description="每侧法兰孔数")
    flange_pcd_left_mm: float = Field(gt=0, description="左法兰孔节圆直径")
    flange_pcd_right_mm: float = Field(gt=0, description="右法兰孔节圆直径")
    center_to_flange_left_mm: float = Field(gt=0, description="花鼓中心到左法兰距离")
    center_to_flange_right_mm: float = Field(gt=0, description="花鼓中心到右法兰距离")
    spoke_hole_diameter_mm: float = Field(default=2.4, gt=0, description="法兰辐条孔直径")


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
    mapping_override: Optional[list[MappingEntry]] = Field(
        default=None, description="可选自定义孔位映射；提供时必须覆盖全部圈孔且不得重复"
    )


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
