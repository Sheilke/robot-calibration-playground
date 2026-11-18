# -*- coding: utf-8 -*-
"""Qt 里嵌 Plotly：一次加载壳页，之后用 ``Plotly.react`` 就地换图。

为什么这么做（已被本机实测验证，见 plans/2026-11-18 的路线 D 验证）
------------------------------------------------------------------
* ``include_plotlyjs=True`` 每张图内联 4.6 MB，且 ``setHtml`` 有 2 MB 上限 →
  改成 ``plotly.min.js`` 落盘一次，壳页用 ``file://`` 加载；
* 每次换图都重载 HTML 会白屏、丢相机视角 → 壳页只加载一次，之后
  ``runJavaScript("window.__setFigure(<fig.to_json()>)")`` 调 ``Plotly.react``；
* 现场无外网 → **不引用 CDN**，``plotly.min.js`` 从本地装载。

标签页与图要双向联动的部分（选中表格行 → 图上高亮）也走这里：``select_points``
调用壳页里的 ``window.__select``；图 → 表的点击事件由壳页通过 ``QWebChannel``
回传（S6 接）。
"""
from __future__ import annotations

import json
from pathlib import Path

from . import paths
from .qt_compat import QWebEngineSettings, QWebEngineView, Qt, QtCore

#: 壳页文件名（落在 ``cache_root()/plot/`` 下，可随时删）。
SHELL_NAME = "plot_shell.html"
#: 与壳页同目录的 plotly.js（一次写入，多次复用）。
PLOTLY_JS_NAME = "plotly.min.js"

#: 壳页模板：``{{PLOTLY_JS}}`` 换成相对路径，``window.__setFigure`` 是换图入口。
_SHELL_TEMPLATE = """<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<title>路点图</title>
<script src="{{PLOTLY_JS}}"></script>
<style>
  html, body { margin: 0; padding: 0; height: 100%; background: #fff; }
  #plot { width: 100%; height: 100%; }
  #empty { font: 14px "Noto Sans CJK SC", "Microsoft YaHei", sans-serif;
           color: #888; padding: 24px; }
</style>
</head>
<body>
<div id="plot"></div>
<div id="empty">还没有数据</div>
<script>
  var gd = null;

  // 换图：首次建，之后 Plotly.react（保留缩放/相机视角）
  window.__setFigure = function (figure) {
    var host = document.getElementById('plot');
    document.getElementById('empty').style.display = 'none';
    var data = figure.data || [];
    var layout = figure.layout || {};
    layout.autosize = true;
    if (gd === null) {
      gd = host;
      Plotly.newPlot(gd, data, layout, {responsive: true, displaylogo: false});
    } else {
      Plotly.react(gd, data, layout, {responsive: true, displaylogo: false});
    }
    return 'ok';
  };

  // 选中若干点（表格行 -> 图上高亮）；indices 是 data[0] 里的点序号
  window.__select = function (indices) {
    if (gd === null) { return 'no-figure'; }
    var sizes = gd.data[0].marker && gd.data[0].marker.size;
    var base = Array.isArray(sizes) ? sizes : [];
    var picked = [];
    var count = (gd.data[0].x || []).length;
    for (var i = 0; i < count; i++) {
      picked.push(indices.indexOf(i) >= 0 ? 12 : (base[i] || 6));
    }
    return Plotly.restyle(gd, {'marker.size': [picked]}).then(function () {
      return 'ok';
    });
  };

  // 供 Qt 侧自检：有没有图、多少点
  window.__state = function () {
    if (gd === null || !gd.data || !gd.data.length) { return 'empty'; }
    return 'points=' + (gd.data[0].x || []).length;
  };
</script>
</body>
</html>
"""


def plotly_js_text() -> str:
    """取当前安装的 plotly 自带 JS（离线用，不联网）。"""
    try:
        from plotly.offline import get_plotlyjs
    except ImportError as error:                 # pragma: no cover - 环境问题
        raise RuntimeError(
            "缺少 plotly：pip install plotly（或改用 requirements-web.txt）") from error
    return get_plotlyjs()


def prepare_shell(force: bool = False) -> Path:
    """把壳页与 ``plotly.min.js`` 写到缓存目录，返回壳页路径。

    只在缺失或体积异常时重写，避免每次启动都写 4.8 MB。

    Args:
        force: 强制重写（plotly 升级后调用）。

    Returns:
        Path: 壳页 HTML 的绝对路径（用 ``file://`` 加载）。
    """
    folder = paths.cache_root() / "plot"
    folder.mkdir(parents=True, exist_ok=True)
    js_path = folder / PLOTLY_JS_NAME
    shell_path = folder / SHELL_NAME

    js_text = plotly_js_text()
    if force or not js_path.is_file() or js_path.stat().st_size < len(js_text) * 0.9:
        js_path.write_text(js_text, encoding="utf-8")
    if force or not shell_path.is_file():
        shell_path.write_text(
            _SHELL_TEMPLATE.replace("{{PLOTLY_JS}}", PLOTLY_JS_NAME),
            encoding="utf-8")
    return shell_path


def figure_payload(figure) -> dict:
    """Plotly Figure -> ``{"data": [...], "layout": {...}}``（可 JSON 化的最小子集）。

    只取 ``data`` 与 ``layout``：``frames`` 用不到，而 ``to_plotly_json()`` 里的
    numpy 数组必须先转成 list，否则 ``json.dumps`` 会抛。
    """
    raw = figure.to_plotly_json()
    return {"data": json.loads(json.dumps(raw.get("data") or [],
                                          cls=_JsonEncoder)),
            "layout": json.loads(json.dumps(raw.get("layout") or {},
                                            cls=_JsonEncoder))}


class _JsonEncoder(json.JSONEncoder):
    """把 numpy 标量/数组转成原生类型（plotly 的 to_plotly_json 仍可能留 numpy）。"""

    def default(self, o):                        # noqa: D401 - 覆写基类
        if hasattr(o, "tolist"):
            return o.tolist()
        if hasattr(o, "item"):
            return o.item()
        return super().default(o)


class PlotView(QWebEngineView):
    """一个 Plotly 视图控件：``load_shell()`` 后即可反复 ``set_figure()``。"""

    #: 壳页加载完成后置位；在此之前调用 ``set_figure`` 会被排队。
    loaded = False

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self._pending = None
        self._last_error = None
        settings = self.settings()
        settings.setAttribute(QWebEngineSettings.JavascriptEnabled, True)
        settings.setAttribute(QWebEngineSettings.LocalContentCanAccessFileUrls,
                              True)
        self.loadFinished.connect(self._on_load_finished)

    # ------------------------------------------------------------------ 加载
    def load_shell(self) -> None:
        """加载（或重载）离线壳页。"""
        shell = prepare_shell()
        self.loaded = False
        self.setUrl(QtCore.QUrl.fromLocalFile(str(shell)))

    def _on_load_finished(self, ok: bool) -> None:
        self.loaded = bool(ok)
        if not ok:
            self._last_error = "壳页加载失败（plotly.min.js 可能没写成功）"
            return
        if self._pending is not None:
            figure, self._pending = self._pending, None
            self.set_figure(figure)
        pending_callback = getattr(self, "_pending_webgl", None)
        if pending_callback is not None:
            self._pending_webgl = None
            self.check_webgl(pending_callback)

    # ------------------------------------------------------------------ 换图
    def set_figure(self, figure) -> None:
        """换图：首图用 ``Plotly.newPlot``，之后走 ``Plotly.react``（视角保留）。

        Args:
            figure: ``plotly.graph_objects.Figure``。
        """
        if not self.loaded:                     # 壳页还没就绪：排队等回调
            self._pending = figure
            return
        payload = figure_payload(figure)
        script = "window.__setFigure(%s)" % json.dumps(payload, ensure_ascii=False)
        self.page().runJavaScript(script, 0, self._on_js_result)

    def select_points(self, indices) -> None:
        """高亮给定序号（表格 → 图上联动）。"""
        if not self.loaded:
            return
        script = "window.__select(%s)" % json.dumps([int(i) for i in indices])
        self.page().runJavaScript(script, 0, self._on_js_result)

    def check_webgl(self, callback=None) -> None:
        """自检 WebGL 是否可用（Plotly 3D 必需）。

        Chromium 在 WebGL 不可用时只会在图区打一句英文提示，现场看不出该怎么办。
        这里主动问一句，由界面把它换成中文的可执行建议。

        Args:
            callback: ``callback(ok: bool, renderer: str)``；``None`` 表示只记录。
        """
        script = """(function(){
          var c = document.createElement('canvas');
          var gl = c.getContext('webgl') || c.getContext('experimental-webgl');
          if (!gl) { return 'NO-WEBGL'; }
          var dbg = gl.getExtension('WEBGL_debug_renderer_info');
          return 'OK|' + (dbg ? gl.getParameter(dbg.UNMASKED_RENDERER_WEBGL) : '');
        })()"""
        if not self.loaded:
            self._pending_webgl = callback
            return

        def _done(result) -> None:
            text = str(result or "")
            ok = text.startswith("OK")
            renderer = text.split("|", 1)[1] if "|" in text else ""
            self._webgl_ok = ok
            self._webgl_renderer = renderer
            if not ok:
                self._last_error = ("WebGL 不可用：Plotly 3D 画不出来。"
                                    "请检查显卡驱动，或改用网页版看图。")
            if callback is not None:
                callback(ok, renderer)

        self.page().runJavaScript(script, 0, _done)

    @property
    def webgl_ok(self):
        """WebGL 自检结果：``True``/``False``/``None``（还没查）。"""
        return getattr(self, "_webgl_ok", None)

    @property
    def webgl_renderer(self) -> str:
        """WebGL 渲染器名（如显卡型号），给状态栏显示。"""
        return getattr(self, "_webgl_renderer", "")

    def _on_js_result(self, result) -> None:
        if result not in (None, "ok"):
            self._last_error = f"JS 返回异常：{result!r}"

    @property
    def last_error(self):
        """最近一次失败原因（给状态栏/日志用）；没失败为 ``None``。"""
        return self._last_error
