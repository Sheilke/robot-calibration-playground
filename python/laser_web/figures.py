# -*- coding: utf-8 -*-
"""交互式 3D 图（plotly）：路点路径 + 失败点 + 段内最差点。

设计文档 §7.4 的性能要求：50 个点**必须**用单个 ``Scatter3d`` trace 着色，
逐点建 trace 会把浏览器拖死。因此这里最多只有 3 个 trace：

1. 成功点：一条 ``Scatter3d``，marker 按 ``color_by`` 连续着色；
2. 失败点：一条 ``Scatter3d``，红色 ``x``；
3. 段内最差点（可选）：一条 ``Scatter3d``，橙色小方块。

本模块只依赖 plotly，不 import 算法，也不 import UI 框架——页面把
``core.run_scan`` 返回的行直接丢进来即可。
"""

from __future__ import annotations

import plotly.graph_objects as go

#: 成功点的路径连线颜色（深灰，避免和 marker 的连续色抢注意力）。
_LINE_COLOR = "#8a94a0"
#: 失败点颜色。
_BAD_COLOR = "#d32f2f"
#: 段内最差点颜色。
_WORST_COLOR = "#f59e0b"
#: ``color_by`` 的合法取值 -> (色标标题, colorbar 标题)。
_COLOR_MODES = {
    "order": ("按路径顺序", "顺序"),
    "margin": ("按关节裕度 min(|J3|,|J5|)", "裕度 °"),
}
#: ISO 五点的立方体棱颜色（灰，不与 P1~P5 抢注意力）。
_CUBE_COLOR = "#9aa3ad"
#: ISO 五点的可解 / 无解 marker 颜色。
_ISO_OK_COLOR = "#2e7d32"
_ISO_BAD_COLOR = "#d32f2f"
#: 立方体的 12 条棱（顶点下标与 :func:`_cube_corners` 的生成顺序对应）。
_CUBE_EDGES = ((0, 1), (0, 2), (0, 4), (1, 3), (1, 5), (2, 3),
               (2, 6), (3, 7), (4, 5), (4, 6), (5, 7), (6, 7))


def _fmt(value, digits: int = 1, suffix: str = "") -> str:
    """None 安全的数字格式化（失败点没有关节角，写 ``n/a``）。"""
    if value is None:
        return "n/a"
    return f"{float(value):.{digits}f}{suffix}"


def _hover_text(row: dict) -> str:
    """单点的 hover 文案（GP / XYZ / |J3| / |J5| / ok）。"""
    position = row.get("pos") or [None, None, None]
    return (f"GP {row.get('gp', '?')}"
            f"<br>X {_fmt(position[0])} mm · Y {_fmt(position[1])} mm"
            f" · Z {_fmt(position[2])} mm"
            f"<br>|J3| {_fmt(row.get('j3_abs'), 3, '°')}"
            f" · |J5| {_fmt(row.get('j5_abs'), 3, '°')}"
            f"<br>ok {bool(row.get('ok'))}")


def _custom_data(row: dict) -> list:
    """``customdata``：GP / XYZ / |J3| / |J5| / ok（数值原样，前端可再算）。"""
    position = list(row.get("pos") or [None, None, None])
    return [row.get("gp"), *position, row.get("j3_abs"), row.get("j5_abs"),
            bool(row.get("ok"))]


def _worst_trace(worsts) -> go.Scatter3d | None:
    """段内最差点的单个 trace（没有可用位置时返回 ``None``）。"""
    points = [item for item in (worsts or []) if item.get("worst_pos")]
    if not points:
        return None
    return go.Scatter3d(
        x=[float(item["worst_pos"][0]) for item in points],
        y=[float(item["worst_pos"][1]) for item in points],
        z=[float(item["worst_pos"][2]) for item in points],
        mode="markers",
        marker={"size": 5, "color": _WORST_COLOR, "symbol": "square",
                "line": {"color": "#7c4a03", "width": 1}},
        name="段内最差点",
        hovertext=[(f"段 GP {item.get('gp_from')}→{item.get('gp_to')}"
                    f"<br>|J3|min {_fmt(item.get('j3_abs_min'), 3, '°')}"
                    f" · |J5|min {_fmt(item.get('j5_abs_min'), 3, '°')}"
                    f"<br>t = {_fmt(item.get('worst_frac'), 3)}")
                   for item in points],
        hovertemplate="%{hovertext}<extra></extra>")


def waypoint_figure(rows, *, color_by: str = "order", worsts=None,
                    all_ok: bool = False, height: int = 620) -> go.Figure:
    """把路点行画成一张交互 3D 图。

    Args:
        rows: ``core.run_scan()["waypoints"]``（字典列表，键 ``gp`` / ``pos`` /
            ``j3_abs`` / ``j5_abs`` / ``ok`` 等）。
        color_by: ``"order"`` 按路径顺序连续着色；``"margin"`` 按
            ``min(|J3|, |J5|)`` 着色（越大越远离奇异）。
        worsts: ``core.run_scan()["segments"]``（可选）；给了就在图上叠一层
            「段内最差点」小方块。
        all_ok: 预览模式（``skip_ik``）没有 IK 结果，所有点按成功绘制。
        height: 画布高度（px）。

    Returns:
        plotly.graph_objects.Figure: 1~3 个 trace；坐标轴单位 mm、等比例
        （``aspectmode="data"``，否则 550×1200×670 mm 的域会被压成方块）。
    """
    mode = color_by if color_by in _COLOR_MODES else "order"
    scale_title, colorbar_title = _COLOR_MODES[mode]
    good = [row for row in rows if all_ok or row.get("ok")]
    bad = [] if all_ok else [row for row in rows if not row.get("ok")]

    figure = go.Figure()
    if good:
        if mode == "order":
            colors = list(range(len(good)))
            colorbar = {"title": colorbar_title, "thickness": 12}
        else:
            colors = [min(row.get("j3_abs") or 0.0, row.get("j5_abs") or 0.0)
                      for row in good]
            colorbar = {"title": colorbar_title, "thickness": 12}
        figure.add_trace(go.Scatter3d(
            x=[float(row["pos"][0]) for row in good],
            y=[float(row["pos"][1]) for row in good],
            z=[float(row["pos"][2]) for row in good],
            mode="markers+lines",
            line={"color": _LINE_COLOR, "width": 2},
            marker={"size": 5, "color": colors, "colorscale": "Viridis",
                    "showscale": True, "colorbar": colorbar,
                    "line": {"color": "#20232a", "width": 0.3}},
            customdata=[_custom_data(row) for row in good],
            hovertext=[_hover_text(row) for row in good],
            hovertemplate="%{hovertext}<extra></extra>",
            name=f"成功 {len(good)} 点 · {scale_title}",
        ))
    if bad:
        figure.add_trace(go.Scatter3d(
            x=[float(row["pos"][0]) for row in bad],
            y=[float(row["pos"][1]) for row in bad],
            z=[float(row["pos"][2]) for row in bad],
            mode="markers",
            marker={"size": 7, "color": _BAD_COLOR, "symbol": "x",
                    "line": {"color": "#7f0000", "width": 0.5}},
            customdata=[_custom_data(row) for row in bad],
            hovertext=[_hover_text(row) for row in bad],
            hovertemplate="%{hovertext}<extra></extra>",
            name=f"无解/未通过 {len(bad)} 点",
        ))
    worst = _worst_trace(worsts)
    if worst is not None:
        figure.add_trace(worst)

    figure.update_layout(
        height=height,
        margin={"l": 0, "r": 0, "t": 8, "b": 0},
        legend={"orientation": "h", "yanchor": "bottom", "y": 1.0,
                "xanchor": "left", "x": 0.0},
        scene={
            "xaxis": {"title": "X (mm)", "backgroundcolor": "#fafafa"},
            "yaxis": {"title": "Y (mm)", "backgroundcolor": "#fafafa"},
            "zaxis": {"title": "Z (mm)", "backgroundcolor": "#fafafa"},
            "aspectmode": "data",
        },
    )
    return figure


def _cube_corners(cube_center, side: float) -> list[list[float]]:
    """立方体的 8 个顶点（顺序与 :data:`_CUBE_EDGES` 的下标对应）。

    这里重算顶点而不是 import 算法模块：画图只要 8 个角，而 ``figures`` 的
    约定是不 import 任何算法（页面传入的都是 ``core`` 已经算好的结果）。
    """
    half = float(side) / 2.0
    center = [float(value) for value in cube_center]
    return [[center[0] + sx * half, center[1] + sy * half,
             center[2] + sz * half]
            for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]


def _cube_trace(cube_center, side: float) -> go.Scatter3d | None:
    """立方体 12 条棱的**单个** trace（段与段之间用 ``None`` 断开）。

    逐条棱建 trace 会多出 12 个图例项，页面上的 P1~P5 会被冲淡；``None``
    在 plotly 的坐标数组里正好表示“断开”，所以一条就够。
    """
    if cube_center is None or side is None or float(side) <= 0.0:
        return None
    corners = _cube_corners(cube_center, side)
    xs: list = []
    ys: list = []
    zs: list = []
    for first, second in _CUBE_EDGES:
        start, end = corners[first], corners[second]
        xs += [start[0], end[0], None]
        ys += [start[1], end[1], None]
        zs += [start[2], end[2], None]
    return go.Scatter3d(x=xs, y=ys, z=zs, mode="lines",
                        line={"color": _CUBE_COLOR, "width": 2},
                        name=f"立方体 L={float(side):g} mm",
                        hoverinfo="skip")


def _iso_hover(row: dict) -> str:
    """ISO 五点的 hover 文案（点名 / XYZ / |J3| / |J5| / cfg / ok）。"""
    position = row.get("pos") or [None, None, None]
    config = row.get("config")
    cfg = "n/a" if config is None else ",".join(str(int(c)) for c in config)
    return (f"{row.get('name', '?')}"
            f"<br>X {_fmt(position[0])} mm · Y {_fmt(position[1])} mm"
            f" · Z {_fmt(position[2])} mm"
            f"<br>|J3| {_fmt(row.get('j3_abs'), 3, '°')}"
            f" · |J5| {_fmt(row.get('j5_abs'), 3, '°')}"
            f"<br>cfg {cfg} · ok {bool(row.get('ok'))}")


def _iso_trace(rows, *, color: str, symbol: str, name: str) -> go.Scatter3d:
    """一组 ISO 五点（可解 / 无解）的单个 trace（marker + 点名标签）。"""
    return go.Scatter3d(
        x=[float(row["pos"][0]) for row in rows],
        y=[float(row["pos"][1]) for row in rows],
        z=[float(row["pos"][2]) for row in rows],
        mode="markers+text",
        marker={"size": 6, "color": color, "symbol": symbol,
                "line": {"color": "#20232a", "width": 0.5}},
        text=[str(row.get("name", "?")) for row in rows],
        textposition="top center",
        textfont={"size": 11, "color": color},
        hovertext=[_iso_hover(row) for row in rows],
        hovertemplate="%{hovertext}<extra></extra>",
        name=name,
    )


def _iso_scene() -> dict:
    """ISO 图的 ``scene`` 配置（坐标轴单位 mm、等比例）。

    每次返回新 dict：plotly 会改写传进来的 layout，共用一个模块级字典迟早
    会串台（本次 figure 的坐标轴设置跑到上一次的 figure 里）。
    """
    return {
        "xaxis": {"title": "X (mm)", "backgroundcolor": "#fafafa"},
        "yaxis": {"title": "Y (mm)", "backgroundcolor": "#fafafa"},
        "zaxis": {"title": "Z (mm)", "backgroundcolor": "#fafafa"},
        "aspectmode": "data",
    }


def iso_cube_figure(points, *, cube_center, side: float, layout: str,
                    plane: str, height: int = 560) -> go.Figure:
    """ISO 9283 五点的 3D 图：立方体 12 条棱 + P1~P5（最多 3 个 trace）。

    trace 预算（设计文档 §7.4 的同一条要求——trace 越多浏览器越卡）：

    1. 立方体的 12 条棱（灰线，``None`` 断段拼成一条）；
    2. 可解点（绿圆 + 点名标签）；
    3. 无解点（红叉 + 点名标签）。

    Args:
        points: ``core.run_iso9283()["points"]``（字典列表，键 ``name`` /
            ``pos`` / ``ok`` / ``j3_abs`` / ``j5_abs`` / ``config``）；空列表
            或 ``None`` 时返回**空 figure**（不抛异常，页面直接画个空白框）。
        cube_center: 立方体中心 ``[x, y, z]``（mm）。
        side: 立方体边长（mm）；``None`` / ``<= 0`` 时只画点。
        layout: 布点方式（写进图例，不参与几何计算）。
        plane: 对角截面（同上）。
        height: 画布高度（px）。

    Returns:
        plotly.graph_objects.Figure: 0~3 个 trace；坐标轴单位 mm、等比例
        （``aspectmode="data"``，否则 550 mm 的立方体会被压扁）。
    """
    rows = [row for row in (points or []) if row.get("pos")]
    figure = go.Figure()
    if not rows:                      # 还没跑：给一个空 figure，让页面画空白
        figure.update_layout(height=height, margin={"l": 0, "r": 0, "t": 8,
                                                    "b": 0})
        return figure

    cube = _cube_trace(cube_center, side)
    if cube is not None:
        figure.add_trace(cube)
    good = [row for row in rows if row.get("ok")]
    bad = [row for row in rows if not row.get("ok")]
    if good:
        figure.add_trace(_iso_trace(
            good, color=_ISO_OK_COLOR, symbol="circle",
            name=f"可解 {len(good)}/{len(rows)} 点 · {layout}/{plane}"))
    if bad:
        figure.add_trace(_iso_trace(
            bad, color=_ISO_BAD_COLOR, symbol="x",
            name=f"无解 {len(bad)} 点"))

    figure.update_layout(
        height=height,
        margin={"l": 0, "r": 0, "t": 8, "b": 0},
        legend={"orientation": "h", "yanchor": "bottom", "y": 1.0,
                "xanchor": "left", "x": 0.0},
        scene=_iso_scene(),
    )
    return figure
