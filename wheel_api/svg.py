"""SVG 穿线图：左右两个法兰视图，标注圈孔号、法兰孔号、阀孔与首根辐条。"""

from __future__ import annotations

import math

from .geometry import hub_angle, rim_angle

RIM_PX = 185.0
PANEL_W = 460.0
HEIGHT = 500.0

COLORS = {
    "trailing": "#2b6cb0",
    "leading": "#dd6b20",
    "radial": "#2f855a",
    "rim": "#444444",
    "hub": "#999999",
    "hole": "#bbbbbb",
    "valve": "#cc0000",
    "first": "#cc0000",
    "text": "#333333",
    "label": "#777777",
}


def _pt(cx, cy, radius, ang):
    return cx + radius * math.cos(ang), cy - radius * math.sin(ang)


def _panel(spec, entries, geometry, side: str, cx: float, cy: float) -> str:
    n_total = spec.rim.holes
    n_side = n_total // 2
    rim_r = spec.rim.erd_mm / 2.0
    flange_r = (spec.hub.flange_pcd_right_mm if side == "right" else spec.hub.flange_pcd_left_mm) / 2.0
    hub_px = RIM_PX * flange_r / rim_r
    side_geom = geometry["sides"][side]
    parts = []

    # 轮圈与法兰
    parts.append(f'<circle cx="{cx}" cy="{cy}" r="{RIM_PX}" fill="none" stroke="{COLORS["rim"]}" stroke-width="3"/>')
    parts.append(f'<circle cx="{cx}" cy="{cy}" r="{hub_px:.1f}" fill="none" stroke="{COLORS["hub"]}" stroke-width="1.5"/>')

    # 辐条（首根 = 0 号圈孔，仅在其所属面板加粗）
    for e in entries:
        if e["side"] != side:
            continue
        a_r = rim_angle(e["rim_hole"], n_total)
        a_h = hub_angle(side, e["hub_hole"], n_side)
        x1, y1 = _pt(cx, cy, hub_px, a_h)
        x2, y2 = _pt(cx, cy, RIM_PX, a_r)
        color = COLORS[e["direction"]]
        dash = ' stroke-dasharray="5 3"' if e["insertion"] == "heads_in" else ""
        width = 2.4 if e["rim_hole"] == 0 else 1.2
        parts.append(
            f'<line x1="{x1:.1f}" y1="{y1:.1f}" x2="{x2:.1f}" y2="{y2:.1f}" '
            f'stroke="{color}" stroke-width="{width}" opacity="0.85"{dash}/>'
        )

    # 圈孔与编号
    for i in range(n_total):
        a = rim_angle(i, n_total)
        x, y = _pt(cx, cy, RIM_PX, a)
        own = (i % 2 == 0) == (side == "right")
        fill = COLORS["text"] if own else COLORS["hole"]
        parts.append(f'<circle cx="{x:.1f}" cy="{y:.1f}" r="2.4" fill="{fill}"/>')
        lx, ly = _pt(cx, cy, RIM_PX + 13, a)
        parts.append(f'<text x="{lx:.1f}" y="{ly:.1f}" class="num" text-anchor="middle" dominant-baseline="middle">{i}</text>')

    # 法兰孔与编号
    for j in range(n_side):
        a = hub_angle(side, j, n_side)
        x, y = _pt(cx, cy, hub_px, a)
        parts.append(f'<circle cx="{x:.1f}" cy="{y:.1f}" r="2" fill="{COLORS["hub"]}"/>')
        lx, ly = _pt(cx, cy, max(hub_px - 11, 8), a)
        parts.append(f'<text x="{lx:.1f}" y="{ly:.1f}" class="hubnum" text-anchor="middle" dominant-baseline="middle">{j}</text>')

    # 阀孔
    v_ang = (spec.rim.valve_position + 0.5) * 2.0 * math.pi / n_total
    vx, vy = _pt(cx, cy, RIM_PX, v_ang)
    parts.append(f'<circle cx="{vx:.1f}" cy="{vy:.1f}" r="5" fill="none" stroke="{COLORS["valve"]}" stroke-width="2"/>')
    lx, ly = _pt(cx, cy, RIM_PX + 26, v_ang)
    parts.append(f'<text x="{lx:.1f}" y="{ly:.1f}" class="valve" text-anchor="middle">VALVE</text>')

    # 首根辐条标记（0 号圈孔在右侧面板）
    if any(e["rim_hole"] == 0 for e in entries):
        a0 = rim_angle(0, n_total)
        fx, fy = _pt(cx, cy, RIM_PX - 14, a0)
        parts.append(f'<circle cx="{fx:.1f}" cy="{fy:.1f}" r="4" fill="{COLORS["first"]}"/>')

    # 标题
    title = f'{side.upper()}  {side_geom["cross"]}x  L={side_geom["spoke_length_mm"]}mm'
    parts.append(f'<text x="{cx}" y="24" class="title" text-anchor="middle">{title}</text>')
    return "\n".join(parts)


def render_svg(spec, entries: list[dict], geometry: dict) -> str:
    width = PANEL_W * 2
    cx1, cx2, cy = PANEL_W / 2, PANEL_W * 1.5, HEIGHT / 2 + 10
    left_entries = [e for e in entries if e["side"] == "left"]
    right_entries = [e for e in entries if e["side"] == "right"]
    legend_y = HEIGHT - 14
    return f'''<svg xmlns="http://www.w3.org/2000/svg" width="{width:.0f}" height="{HEIGHT:.0f}" viewBox="0 0 {width:.0f} {HEIGHT:.0f}">
<style>
  text {{ font-family: sans-serif; fill: {COLORS["text"]}; }}
  .title {{ font-size: 14px; font-weight: bold; }}
  .num {{ font-size: 8px; fill: {COLORS["label"]}; }}
  .hubnum {{ font-size: 7px; fill: {COLORS["label"]}; }}
  .valve {{ font-size: 9px; fill: {COLORS["valve"]}; font-weight: bold; }}
  .legend {{ font-size: 10px; }}
</style>
<rect width="100%" height="100%" fill="white"/>
{_panel(spec, right_entries, geometry, "right", cx1, cy)}
{_panel(spec, left_entries, geometry, "left", cx2, cy)}
<g class="legend">
  <line x1="20" y1="{legend_y}" x2="50" y2="{legend_y}" stroke="{COLORS["trailing"]}" stroke-width="2"/>
  <text x="56" y="{legend_y + 3}">trailing</text>
  <line x1="120" y1="{legend_y}" x2="150" y2="{legend_y}" stroke="{COLORS["leading"]}" stroke-width="2"/>
  <text x="156" y="{legend_y + 3}">leading</text>
  <line x1="220" y1="{legend_y}" x2="250" y2="{legend_y}" stroke="{COLORS["text"]}" stroke-width="2" stroke-dasharray="5 3"/>
  <text x="256" y="{legend_y + 3}">heads-in</text>
  <circle cx="330" cy="{legend_y}" r="4" fill="{COLORS["first"]}"/>
  <text x="340" y="{legend_y + 3}">first spoke</text>
  <circle cx="420" cy="{legend_y}" r="5" fill="none" stroke="{COLORS["valve"]}" stroke-width="2"/>
  <text x="430" y="{legend_y + 3}">valve</text>
</g>
</svg>'''
