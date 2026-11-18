# -*- coding: utf-8 -*-
"""Qt 里嵌 Plotly：一次加载壳页，之后用 ``Plotly.react`` 就地换图。

为什么这么做（已被本机实测验证，见 plans/2026-11-18 的路线 D 验证）
------------------------------------------------------------------
* ``include_plotlyjs=True`` 每张图内联 4.6 MB，且 ``setHtml`` 有 2 MB 上限 →
  改成 ``plotly.min.js`` 落盘一次，壳页用 ``file://`` 加载；
* 每次换图都重载 HTML 会白屏、丢相机视角 → 壳页只加载一次，之后
  ``runJavaScript("window.__setFigure(<fig.to_json>())")`` 调 ``Plotly.react``；
* 现场无外网 → **不引用 CDN**，``plotly.min.js`` 与 ``qwebchannel.js`` 都从
  本地装载（后者从 Qt 资源里读出来落盘，``qrc://`` 在 ``file://`` 页面上不可靠）。

S6 补上的两件事
--------------
1. **表格 ↔ 图双向联动**：``select_points`` 调壳页的 ``window.__select``（按
   ``customdata[0]`` 匹配，生成/修复是 GP 号、ISO 是点名）；图上的点击经
   ``QWebChannel`` 回传成 :attr:`PlotView.pointClicked`。
2. **性能与进程可测**：:attr:`PlotView.load_count` 计数"壳页加载了几次"（换图
   绝不重载页面），:meth:`PlotView.measure_react` 回传 JS 侧
   ``Plotly.react`` 的真实毫秒数（**只计 react 本身**，相机 ``relayout`` 与点击
   绑定在计时之后才做）；:class:`PlotHost` 让 ``PlotView`` 到"真要看图"时才
   创建（首启不拉起 Chromium 子进程）。
"""
from __future__ import annotations

import json
import time
from pathlib import Path

from . import paths
from .qt_compat import (QWebChannel, QWebEngineSettings, QWebEngineView, Qt,
                        QtCore, QtWidgets, Signal, Slot)


def flush_deferred_deletes() -> None:
    """立即处理 ``deleteLater()``（不等事件循环）。

    没有 ``app.exec()`` 的场景（离屏测试 / 冒烟脚本 / 关窗那一刻）里，Qt 只在
    **进入事件循环**时才处理 ``DeferredDelete``。所以 ``deleteLater()`` 之后
    页面可能一直活到进程退出——profile 先被释放，于是打出
    "Release of profile requested but WebEnginePage still not deleted"。
    """
    app = QtWidgets.QApplication.instance()
    if app is None:                             # pragma: no cover - 无 app
        return
    event = getattr(QtCore.QEvent, "DeferredDelete", None)
    if event is None:                           # pragma: no cover - 绑定差异
        app.processEvents()
        return
    app.sendPostedEvents(None, event)
    app.processEvents()

#: 壳页文件名（落在 ``cache_root()/plot/`` 下，可随时删）。
SHELL_NAME = "plot_shell.html"
#: 与壳页同目录的 plotly.js（一次写入，多次复用）。
PLOTLY_JS_NAME = "plotly.min.js"
#: 与壳页同目录的 Qt WebChannel 客户端（从 ``:/qtwebchannel/qwebchannel.js``
#: 拷出来：``file://`` 页面读 ``qrc://`` 不一定被允许）。
WEBCHANNEL_JS_NAME = "qwebchannel.js"
#: JS 里 QWebChannel 的注册名（Python 侧 ``registerObject`` 用同一个）。
BRIDGE_NAME = "plotBridge"
#: 选中点的 marker 边长（像素）；未选中用图里原来的大小。
SELECT_SIZE = 12

#: 壳页模板：``{{PLOTLY_JS}}`` / ``{{QWEBCHANNEL_JS}}`` 换成相对路径，
#: ``window.__setFigure`` 是换图入口。
_SHELL_TEMPLATE = """<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<title>路点图</title>
<script src="{{QWEBCHANNEL_JS}}"></script>
<script src="{{PLOTLY_JS}}"></script>
<style>
  html, body { margin: 0; padding: 0; height: 100%; background: #fff;
               position: relative; }
  #plot { width: 100%; height: 100%; }
  /* 绝对定位：不然它排在 100% 高的 #plot 下面，永远落在视口之外（看不见）。 */
  #empty { position: absolute; top: 0; left: 0; right: 0;
           font: 14px "Noto Sans CJK SC", "Microsoft YaHei", sans-serif;
           color: #888; padding: 24px; }
</style>
</head>
<body>
<div id="plot"></div>
<div id="empty">还没有数据</div>
<script>
  var gd = null;
  var bridge = null;
  var clicksBound = false;
  var selected = [];
  // 每个 trace 的**原始**点尺寸（数字数组，与 customdata 一一对应）。
  // Plotly.restyle 是**就地**改 gd.data[t].marker.size 的，所以基准必须来自
  // 换图时传进来的 figure；回头读 trace.marker.size 就会把上次高亮的 12 当成
  // 新基准，于是选过的点再也缩不回去（S6 评审 R1 的实测）。
  window.__baseSizes = [];
  // 最近一次换图的 JS 侧耗时（PlotView.measure_react 回读它）。
  // 口径：只含 Plotly.newPlot/react 本身，**不含**相机 relayout 与点击绑定。
  window.__lastReact = null;

  // 从 figure.data 拍一份点尺寸快照（标量 -> 每点一个；缺省 -> Plotly 默认 6）。
  function baseSizesOf(data) {
    var result = [];
    for (var t = 0; t < data.length; t++) {
      var trace = data[t] || {};
      var points = trace.customdata || [];
      var count = points.length || (trace.x || []).length;
      var size = (trace.marker || {}).size;
      var sizes = [];
      for (var j = 0; j < count; j++) {
        var value = Array.isArray(size) ? size[j] : size;
        sizes.push((value === undefined || value === null) ? 6 : value);
      }
      result.push(sizes);
    }
    return result;
  }

  if (typeof QWebChannel !== 'undefined' && typeof qt !== 'undefined'
      && qt !== null && qt.webChannelTransport) {
    new QWebChannel(qt.webChannelTransport, function (channel) {
      bridge = (channel.objects && channel.objects.{{BRIDGE_NAME}}) || null;
    });
  }

  // customdata[0] 是联动用的键：生成/修复 = GP 号，ISO = P1..P5
  function keyOf(customdata) {
    if (customdata === undefined || customdata === null) { return null; }
    if (Array.isArray(customdata)) {
      return customdata.length ? customdata[0] : null;
    }
    return customdata;
  }

  function notifySelection(keys) {
    if (bridge && typeof bridge.selectFromPlot === 'function') {
      bridge.selectFromPlot(JSON.stringify(keys.map(String)));
    }
  }

  function bindClicks() {
    if (clicksBound || gd === null || typeof gd.on !== 'function') { return; }
    gd.on('plotly_click', function (event) {
      if (!event || !event.points || !event.points.length) { return; }
      var keys = [];
      for (var i = 0; i < event.points.length; i++) {
        var key = keyOf(event.points[i].customdata);
        if (key !== null && key !== undefined) { keys.push(String(key)); }
      }
      if (keys.length) { notifySelection(keys); }
    });
    clicksBound = true;
  }

  function now() {
    return (window.performance && performance.now) ? performance.now() : Date.now();
  }

  // 换图：首次建，之后 Plotly.react（保留缩放/相机视角）。
  // token 只是给 Python 侧认领这次耗时用的，可以省略。
  window.__setFigure = function (figure, token) {
    var host = document.getElementById('plot');
    var empty = document.getElementById('empty');
    var data = figure.data || [];
    var layout = figure.layout || {};
    layout.autosize = true;
    var started = now();
    // 基准只从传进来的 figure 里拍（restyle 之后 gd.data 已经不可信）。
    // 在这里（Plotly 调用之前）就拍好，紧跟着来的 __select 才有基准可用。
    window.__baseSizes = baseSizesOf(data);
    // 新图一律“没有选中点”。不能放到 done() 里复位：done() 是异步的，会把
    // 紧随换图之后的那次 __select 一起抹掉（表里选着、图上却不亮）。
    selected = [];
    if (!data.length) {
      // 空图（这一页还没出过图）：清掉上一页的残留，重新露出「还没有数据」。
      if (gd !== null) {
        try { Plotly.purge(gd); } catch (error) { /* 画布本来就是空的 */ }
        gd = null;
        clicksBound = false;
      }
      empty.style.display = 'block';
      window.__lastReact = {
        ms: now() - started,
        token: (token === undefined) ? null : token,
        first: true,
        points: 0
      };
      return 'ok';
    }
    empty.style.display = 'none';
    var first = (gd === null);
    // 相机：Plotly.react 在 layout 里带了 scene（我们的图都有）时会回到默认视角。
    // 换图不该让现场调好的视角跳回去，所以先把当前相机存下来、react 完再放回去。
    var camera = null;
    if (!first && gd._fullLayout && gd._fullLayout.scene
        && gd._fullLayout.scene.camera) {
      try {
        camera = JSON.parse(JSON.stringify(gd._fullLayout.scene.camera));
      } catch (error) { camera = null; }
    }
    var promise;
    if (first) {
      gd = host;
      promise = Plotly.newPlot(gd, data, layout,
                               {responsive: true, displaylogo: false});
    } else {
      promise = Plotly.react(gd, data, layout,
                             {responsive: true, displaylogo: false});
    }
    var done = function () {
      window.__lastReact = {
        ms: now() - started,
        token: (token === undefined) ? null : token,
        first: first,
        points: data.length ? (data[0].x || []).length : 0
      };
      // newPlot/react 之后重建基准（同一个来源：传进来的 data）。
      window.__baseSizes = baseSizesOf(data);
      if (camera) { Plotly.relayout(gd, {'scene.camera': camera}); }
      bindClicks();
    };
    if (promise && typeof promise.then === 'function') {
      promise.then(done);
    } else {
      done();
    }
    return 'ok';
  };

  // 选中若干点（表格行 -> 图上放大）。keys 是 customdata[0] 的集合。
  // silent=true 时不再回传 Qt（避免表格 -> 图 -> 表格 的来回循环）。
  window.__select = function (keys, silent) {
    if (gd === null) { return 'no-figure'; }
    var list = keys || [];
    var wanted = {};
    for (var i = 0; i < list.length; i++) { wanted[String(list[i])] = true; }
    selected = [];
    for (var s = 0; s < list.length; s++) { selected.push(String(list[s])); }

    for (var t = 0; t < gd.data.length; t++) {
      var trace = gd.data[t];
      var points = trace.customdata || [];
      if (!points.length) { continue; }          // 立方体棱线之类没有键的 trace
      // 基准只认换图时的快照；没有快照（半张图/旧壳页）才退回默认 6。
      // 绝不能读 trace.marker.size——Plotly.restyle 已经把它改成含 12 的了。
      var base = window.__baseSizes[t] || [];
      var sizes = [];
      for (var j = 0; j < points.length; j++) {
        var key = keyOf(points[j]);
        var fallback = (base[j] === undefined) ? 6 : base[j];
        sizes.push(wanted[String(key)] ? {{SELECT_SIZE}} : fallback);
      }
      Plotly.restyle(gd, {'marker.size': [sizes]}, [t]);
    }
    if (!silent) { notifySelection(selected); }
    // 不返回 Promise：Qt 的 runJavaScript 只能序列化简单值，Promise 回来是 {}，
    // 会被当成"JS 返回异常"记进 last_error。
    return 'ok';
  };

  // 供 Qt 侧自检：有没有图、多少点
  window.__state = function () {
    if (gd === null || !gd.data || !gd.data.length) { return 'empty'; }
    return 'points=' + (gd.data[0].x || []).length;
  };

  // 供 Qt 侧自检：当前选了哪些键
  window.__selectedKeys = function () { return JSON.stringify(selected); };

  // 供 Qt 侧自检：每个 trace 现在的 marker.size（没设过就是 null）。
  // 高亮只能靠读这个数组证明（__selectedKeys 只说明"以为选了谁"）。
  window.__markerSizes = function () {
    if (gd === null || !gd.data) { return '[]'; }
    var result = [];
    for (var t = 0; t < gd.data.length; t++) {
      var marker = gd.data[t].marker || {};
      result.push(marker.size === undefined ? null : marker.size);
    }
    return JSON.stringify(result);
  };

  // 供 Qt 侧自检：「还没有数据」那行字到底看不看得见
  window.__emptyShown = function () {
    return document.getElementById('empty').style.display === 'block'
        ? 'shown' : 'hidden';
  };

  // 供 Qt 侧自检：每个 trace 的键（P1 的第 0 个键必须存在）
  window.__keys = function () {
    if (gd === null || !gd.data) { return '[]'; }
    var result = [];
    for (var t = 0; t < gd.data.length; t++) {
      var points = gd.data[t].customdata || [];
      var keys = [];
      for (var j = 0; j < points.length; j++) { keys.push(keyOf(points[j])); }
      result.push(keys);
    }
    return JSON.stringify(result);
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


def webchannel_js_text() -> str:
    """从 Qt 资源里取 ``qwebchannel.js``（拿不到时返回空串）。

    QtWebChannel 模块没装/资源没注册时返回空串：壳页里的 ``QWebChannel``
    就是 ``undefined``，图照样能画，只是"点图 -> 表格"这条联动不可用
    （状态栏会给一句话，见 :attr:`PlotView.link_warning`）。
    """
    try:
        file = QtCore.QFile(":/qtwebchannel/qwebchannel.js")
        if not file.exists() or not file.open(QtCore.QIODevice.ReadOnly):
            return ""
        try:
            return bytes(file.readAll()).decode("utf-8")
        finally:
            file.close()
    except (AttributeError, RuntimeError):       # pragma: no cover - 环境问题
        return ""


def prepare_shell(force: bool = False) -> Path:
    """把壳页、``plotly.min.js``、``qwebchannel.js`` 写到缓存目录，返回壳页路径。

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
    channel_path = folder / WEBCHANNEL_JS_NAME

    js_text = plotly_js_text()
    if force or not js_path.is_file() or js_path.stat().st_size < len(js_text) * 0.9:
        js_path.write_text(js_text, encoding="utf-8")
    channel_text = webchannel_js_text()
    if channel_text and (force or not channel_path.is_file()
                         or channel_path.stat().st_size != len(channel_text)):
        channel_path.write_text(channel_text, encoding="utf-8")
    html = (_SHELL_TEMPLATE.replace("{{PLOTLY_JS}}", PLOTLY_JS_NAME)
                            .replace("{{QWEBCHANNEL_JS}}", WEBCHANNEL_JS_NAME)
                            .replace("{{BRIDGE_NAME}}", BRIDGE_NAME)
                            .replace("{{SELECT_SIZE}}", str(SELECT_SIZE)))
    # 壳页只有 5 KB，逐字比一次比“文件存在就跳过”可靠：模板改了（S6 就是）
    # 而缓存里还是旧壳页的话，界面看上去一切正常、只是联动/计时全不生效。
    if (force or not shell_path.is_file()
            or shell_path.read_text(encoding="utf-8") != html):
        shell_path.write_text(html, encoding="utf-8")
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


class _PlotBridge(QtCore.QObject):
    """JS -> Python 的唯一入口（``QWebChannel`` 对象）。

    Signals:
        selection: JSON 数组字符串（``["5"]`` / ``["P1"]``）。

    注意：这里的槽是 Qt 从 JS 侧调进来的，**抛异常不能穿出去**——PyQt5 下
    会当致命错误直接 abort 进程（图 → 表联动不值得为一条坏数据带走窗口）。
    """

    selection = Signal(str)

    @Slot(str)
    def selectFromPlot(self, payload: str) -> None:
        """壳页里点了图上的点（或 JS 主动调了 ``__select``）。"""
        try:
            self.selection.emit(str(payload or "[]"))
        except Exception:                       # noqa: BLE001 - 槽里不能抛
            view = self.parent()
            if view is not None:
                view._last_error = "图 → 表联动处理失败（已忽略，界面照常）"


class PlotView(QWebEngineView):
    """一个 Plotly 视图控件：``load_shell()`` 后即可反复 ``set_figure()``。

    Signals:
        pointClicked: 图上点了点（参数是 ``customdata[0]`` 的列表）。
    """

    #: 壳页加载完成后置位；在此之前调用 ``set_figure`` 会被排队。
    loaded = False
    #: 图上点了点：参数是键的列表（``["5"]`` / ``["P1"]``）。
    pointClicked = Signal(object)

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self.loaded = False
        self.load_count = 0
        self._pending = None
        self._pending_measure = None
        self._pending_webgl = None
        self._last_error = None
        self._last_figure = None
        self._selected = []
        self._react_token = 0
        self._measure_callbacks = {}
        self._shutting_down = False
        settings = self.settings()
        settings.setAttribute(QWebEngineSettings.JavascriptEnabled, True)
        settings.setAttribute(QWebEngineSettings.LocalContentCanAccessFileUrls,
                              True)
        self.loadFinished.connect(self._on_load_finished)
        self._bridge = None
        self._channel = None
        self._setup_channel()

    # ------------------------------------------------------------------ 关闭
    def shutdown(self) -> None:
        """显式拆掉这个视图：**先删 page，再删 view**（顺序不能反）。

        为什么必须显式：``QWebEngineProfile`` 是全局单例，关窗口时如果还有
        ``QWebEnginePage`` 活着，Qt 就报
        ``Release of profile requested but WebEnginePage still not deleted.
        Expect troubles !``——交付到 Windows 上就是"跑着图关窗报错 / 退出崩"。
        裸 ``deleteLater()`` 不够：没有事件循环（测试、冒烟、关窗那一刻）时
        ``DeferredDelete`` 根本不处理（实测 5 个视图 → 5 条告警）。

        顺序反了等于没修：先删 view 的话，page 还挂在已经没了的视图上。
        幂等：重复调、或对象已经被删调，都不报错。
        """
        if getattr(self, "_shutting_down", False):
            return
        self._shutting_down = True
        self.loaded = False
        self._pending = None
        self._pending_measure = None
        self._last_figure = None
        self._measure_callbacks = {}
        try:
            self.stop()                         # 别在拆的时候还回调 JS
        except RuntimeError:                    # pragma: no cover - 已删
            pass
        page = None
        try:
            page = self.page()
        except RuntimeError:                    # pragma: no cover - 已删
            page = None
        if page is not None:
            try:
                page.setWebChannel(None)        # 通道会把 bridge 一起留住
            except RuntimeError:                # pragma: no cover - 已删
                pass
            page.deleteLater()
            flush_deferred_deletes()            # page 真正析构（关键）
        self._channel = None
        self._bridge = None
        try:
            self.setParent(None)
            self.deleteLater()
        except RuntimeError:                    # pragma: no cover - 已删
            return
        flush_deferred_deletes()                # view 也跟着没

    # ------------------------------------------------------------------ 通道
    def _setup_channel(self) -> None:
        """装 ``QWebChannel``（没有 QtWebChannel 时静默降级：图照画，联动没有）。"""
        if QWebChannel is None:                  # pragma: no cover - 环境问题
            return
        self._bridge = _PlotBridge(self)
        self._channel = QWebChannel(self.page())
        self._channel.registerObject(BRIDGE_NAME, self._bridge)
        self.page().setWebChannel(self._channel)
        self._bridge.selection.connect(self._on_bridge_selection)

    def _on_bridge_selection(self, payload: str) -> None:
        """把 JS 传来的 JSON 数组解成 Python 列表，再发给页面。

        两道防护（S6 评审建议 4）：

        * **只收 ``str`` / ``int`` 键**：``customdata`` 是我们自己写的，但桥是
          外部输入（页面里任何脚本都能调），形状不对的就不该进 ``pointClicked``；
        * **异常不外抛**：本槽（含随后的 ``pointClicked``）由 Qt 从 JS 侧调用，
          PyQt5 下槽里抛异常会当致命错误直接 abort 进程。一条坏数据不能把窗口
          带走（错误记进 :attr:`last_error`，下次状态栏会说一句话）。
        """
        if getattr(self, "_shutting_down", False):
            return
        try:
            keys = json.loads(payload) if payload else []
            if not isinstance(keys, list):
                keys = [keys]
            clean = []
            for key in keys:
                if isinstance(key, bool) or not isinstance(key, (int, str)):
                    continue                    # 字典/列表/None 一律不信
                text = str(key).strip()
                if text:
                    clean.append(text)
            self._selected = clean
            if clean:
                self.pointClicked.emit(list(clean))
        except Exception:                       # noqa: BLE001 - 槽里不能抛
            self._selected = []
            self._last_error = "图 → 表联动收到坏数据（已忽略，界面照常）"

    @property
    def link_warning(self) -> str:
        """图 → 表联动不可用时的原因（空串表示没毛病）。"""
        if QWebChannel is None:
            return "缺 QtWebChannel：点图不会选中表格行（图本身照常）"
        if not webchannel_js_text():
            return "缺 qwebchannel.js：点图不会选中表格行（图本身照常）"
        return ""

    # ------------------------------------------------------------------ 加载
    def load_shell(self) -> None:
        """加载（或重载）离线壳页。"""
        if getattr(self, "_shutting_down", False):
            return
        shell = prepare_shell()
        self.loaded = False
        self.setUrl(QtCore.QUrl.fromLocalFile(str(shell)))

    def _on_load_finished(self, ok: bool) -> None:
        if getattr(self, "_shutting_down", False):
            return
        self.load_count += 1
        self.loaded = bool(ok)
        if not ok:
            self._last_error = "壳页加载失败（plotly.min.js 可能没写成功）"
            return
        pending_measure, self._pending_measure = self._pending_measure, None
        if self._pending is not None:
            figure, self._pending = self._pending, None
            if (pending_measure is not None
                    and pending_measure[0] in self._measure_callbacks):
                # measure_react 在壳页加载之前就被调了：现在才真推（带 token）
                # 并开始轮询。否则调用方的 callback 永远不会被回调——界面会
                # 一直等一个不会到的数字（S6 评审建议 3）。
                token, deadline = pending_measure
                self._run_figure(figure, token)
                QtCore.QTimer.singleShot(
                    10, lambda: self._poll_react(token, deadline))
            else:
                self.set_figure(figure)
        elif (pending_measure is not None
                and pending_measure[0] in self._measure_callbacks):
            # 排队的那张图被别的 set_figure 顶掉了：计时照样要收尾（超时
            # 给 None），不能让调用方永久等。
            token, deadline = pending_measure
            QtCore.QTimer.singleShot(
                10, lambda: self._poll_react(token, deadline))
        pending_callback = self._pending_webgl
        if pending_callback is not None:
            self._pending_webgl = None
            self.check_webgl(pending_callback)

    # ------------------------------------------------------------------ 换图
    def set_figure(self, figure) -> None:
        """换图：首图用 ``Plotly.newPlot``，之后走 ``Plotly.react``（视角保留）。

        **同一张图不重复推**：三个页面都往这一个视图推图（切页时主窗口推、
        页面自己去抖也推），而 ``Plotly.react`` 会把相机视角与 marker 尺寸重置
        回原值。同一对象再推一次只会把现场调好的视角/高亮打回默认值，所以
        直接跳过。

        Args:
            figure: ``plotly.graph_objects.Figure``。
        """
        if not self.loaded:                     # 壳页还没就绪：排队等回调
            self._pending = figure
            return
        if getattr(self, "_shutting_down", False):
            return
        if figure is self._last_figure:          # 已经推过这一张
            return
        self._last_figure = figure
        self._run_figure(figure, None)

    def _run_figure(self, figure, token) -> None:
        """真正把图推给壳页（``token`` 有值时壳页会记下这次耗时）。"""
        payload = figure_payload(figure)
        args = json.dumps(payload, ensure_ascii=False)
        script = ("window.__setFigure(%s)" % args if token is None
                  else "window.__setFigure(%s, %d)" % (args, int(token)))
        self.page().runJavaScript(script, 0, self._on_js_result)

    def measure_react(self, figure, callback=None, timeout_s: float = 5.0):
        """推一张图并回传 JS 侧 ``Plotly.newPlot``/``Plotly.react`` 的耗时。

        计时在壳页里用 ``performance.now()`` 取，从调用 Plotly 到它的 promise
        resolve（含真实绘制）。这是 P3 的唯一可信口径：Python 侧的
        ``runJavaScript`` 往返包含了排队时间，不能当绘制耗时。

        **口径警告（S6 评审建议 5）**：这个数字只是 ``react`` 本身——
        相机 ``relayout`` 与点击绑定（``bindClicks``）都在计时之后才做，**不**
        包含在里面。S7 别把它当"整帧耗时"用。

        Args:
            figure: 要推的图。
            callback: ``callback(毫秒: float | None)``（超时给 ``None``）。
            timeout_s: 等 JS 回报的超时（秒）。

        Returns:
            int: 本次的 token（同一张图可以连续测多次）。
        """
        self._react_token += 1
        token = self._react_token
        if callback is not None:
            self._measure_callbacks[token] = callback
        if not self.loaded:
            # 壳页还没好：记下 token/截至时间，_on_load_finished 里补跑（不再
            # 只排队不管，那样调用方的 callback 永远不会来）。
            self._pending = figure
            self._pending_measure = (token, time.monotonic() + timeout_s)
            return token
        self._run_figure(figure, token)
        if callback is not None:
            QtCore.QTimer.singleShot(
                10, lambda: self._poll_react(token, time.monotonic() + timeout_s))
        return token

    def _poll_react(self, token: int, deadline: float) -> None:
        """轮询壳页里的 ``window.__lastReact``，认领属于本次 token 的耗时。"""
        callback = self._measure_callbacks.get(token)
        if callback is None:
            return
        script = ("window.__lastReact === null ? 'null' "
                  ": JSON.stringify(window.__lastReact)")

        def _done(raw) -> None:
            try:
                data = json.loads(raw) if raw and raw != "null" else None
            except (TypeError, ValueError):
                data = None
            if isinstance(data, dict) and int(data.get("token") or -1) == token:
                self._measure_callbacks.pop(token, None)
                callback(float(data.get("ms") or 0.0))
                return
            if time.monotonic() > deadline:
                self._measure_callbacks.pop(token, None)
                callback(None)
                return
            QtCore.QTimer.singleShot(15, lambda: self._poll_react(token, deadline))

        self.page().runJavaScript(script, 0, _done)

    def select_points(self, keys, notify: bool = False) -> None:
        """按 ``customdata[0]`` 高亮给定键（表格 → 图上联动）。

        Args:
            keys: 键的列表（GP 号或点名）。字符串/数字都收。
            notify: 是否让壳页再回传一次点击（默认不回传，避免表格↔图死循环）。
        """
        if not self.loaded:
            return
        payload = [str(key) for key in keys]
        self._selected = payload
        script = "window.__select(%s, %s)" % (
            json.dumps(payload, ensure_ascii=False),
            "true" if not notify else "false")
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


class PlotHost(QtWidgets.QWidget):
    """懒创建的 ``PlotView`` 容器（P5：没图要看就不启动 Chromium）。

    三个有图的页面共用同一个实例。构造时不建 ``QWebEngineView``；第一次
    :meth:`set_figure`（或显式 :meth:`ensure`）才创建、加载壳页、跑 WebGL
    自检。这样：首启只看表单时没有 ``QtWebEngineProcess``；页面数量与进程数
    无关（永远只有一个视图）。

    Signals:
        loadFinished: 转发 ``PlotView.loadFinished``（冒烟脚本等它）。
        pointClicked: 转发 ``PlotView.pointClicked``。
    """

    #: 三页共用一个实例：由主窗口在切页时搬进当前页的 ``PlotSlot``，
    #: 页面自己**不要**把它 addWidget 到自己的布局里（会从上一页被偷偷移走）。
    SHARED_ACROSS_PAGES = True

    loadFinished = Signal(bool)
    pointClicked = Signal(object)

    def __init__(self, parent=None, on_webgl=None) -> None:
        """建容器。

        Args:
            parent: 父控件。
            on_webgl: WebGL 自检回调 ``callback(ok, renderer)``；在视图真正
                创建后调用一次。
        """
        super().__init__(parent)
        self._view = None
        self._on_webgl = on_webgl
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        self._placeholder = QtWidgets.QLabel(
            "第一次需要看图时才启动图形引擎（离线，不联网）。")
        self._placeholder.setAlignment(Qt.AlignCenter)
        self._placeholder.setWordWrap(True)      # 不然它的宽度会顶住整窗最小宽度
        self._placeholder.setStyleSheet("color:#888")
        layout.addWidget(self._placeholder)

    def ensure(self) -> "PlotView":
        """确保 ``PlotView`` 存在（创建 + 加载壳页），返回它。"""
        if self._view is not None:
            return self._view
        view = PlotView(self)
        view.loadFinished.connect(self.loadFinished.emit)
        view.pointClicked.connect(self.pointClicked.emit)
        self.layout().addWidget(view)
        self._placeholder.hide()
        self._view = view
        view.load_shell()
        if self._on_webgl is not None:
            view.check_webgl(self._on_webgl)
        return view

    @property
    def view(self):
        """已创建的 ``PlotView``（没创建时为 ``None``，不会触发创建）。"""
        return self._view

    def shutdown(self) -> None:
        """拆掉共享视图（主窗口 ``closeEvent`` 调，三页只此一个）。

        之后再 :meth:`set_figure` 会重新建一个视图（"关窗 → 再开"可测），
        但**不要**在关窗后继续用旧的那个：它已经被删了。
        """
        view, self._view = self._view, None
        if view is None:
            return
        layout = self.layout()
        if layout is not None:
            layout.removeWidget(view)
        view.shutdown()
        self._placeholder.show()

    @property
    def created(self) -> bool:
        """是否已经建了真视图（P5 的断言点）。"""
        return self._view is not None

    def set_figure(self, figure) -> None:
        """换图（第一次调用会把视图建出来）。"""
        self.ensure().set_figure(figure)

    def select_points(self, keys, notify: bool = False) -> None:
        """高亮若干键；视图还没建时什么都不做（没有图可高亮）。"""
        if self._view is not None:
            self._view.select_points(keys, notify=notify)

    def measure_react(self, figure, callback=None, timeout_s: float = 5.0):
        """计时换图（见 :meth:`PlotView.measure_react`）。"""
        return self.ensure().measure_react(figure, callback, timeout_s)

    def page(self):
        """壳页对象（会确保视图存在）。"""
        return self.ensure().page()

    def check_webgl(self, callback=None) -> None:
        """WebGL 自检；视图还没建时先记下回调，建好后自动问一次。"""
        if self._view is not None:
            self._view.check_webgl(callback)
        else:
            self._on_webgl = callback

    @property
    def loaded(self) -> bool:
        """壳页是否已加载完成（没建视图时为 ``False``）。"""
        return bool(self._view is not None and self._view.loaded)

    @property
    def load_count(self) -> int:
        """壳页加载次数（换图不该让它增加）。"""
        return 0 if self._view is None else int(self._view.load_count)

    @property
    def webgl_ok(self):
        """WebGL 自检结果（没建视图时 ``None``）。"""
        return None if self._view is None else self._view.webgl_ok

    @property
    def webgl_renderer(self) -> str:
        """WebGL 渲染器名（没建视图时空串）。"""
        return "" if self._view is None else self._view.webgl_renderer

    @property
    def last_error(self):
        """最近一次失败原因。"""
        return None if self._view is None else self._view.last_error

    @property
    def link_warning(self) -> str:
        """图 → 表联动是否可用（视图没建时也检查，见 PlotView.link_warning）。

        结果缓存在实例上：``webchannel_js_text()`` 要读 16 KB 的 Qt 资源，而
        状态栏会反复刷新。
        """
        cached = getattr(self, "_link_warning", None)
        if cached is None:
            if QWebChannel is None or not webchannel_js_text():
                cached = ("缺 QtWebChannel/qwebchannel.js：点图不会选中表格行"
                          "（图本身照常）")
            else:
                cached = ""
            self._link_warning = cached
        return cached
