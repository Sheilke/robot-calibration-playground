# -*- coding: utf-8 -*-
"""三页共用的右侧布局与「表格 ↔ 图」联动小工具（S6）。

为什么单独一层
--------------
生成 / 修复 / ISO 三页的右半边是同一个结构：**预览卡 / 图 / 结果**。
S3–S5 是各页自己摆 ``QVBoxLayout`` + 两段 ``QSplitter``，结果在 1180×780 的
窗口里，预览卡按 sizeHint 拿走固定高度、剩下的被图和结果对半分——表格体只剩
表头加一行（S6 的 L1，现场最先抱怨的就是这个）。

这里把"三段分割 + 比例记忆 + 结果表最小高度"做成一份，三页都调它：

* :class:`RightSplitter`：垂直三段，首开时预览卡按**内容高度**拿（150–220，
  见 :func:`preview_default_height`）、剩下的按 :data:`DEFAULT_SPLIT_RATIOS`
  折算给图 / 结果；用户拖动后把比例写进 ``QSettings``，下次开窗读回来；
* :class:`BodySplitter`：左右两栏（表单列 / 右栏）之间的**横向**分割器——
  2026-02 之前这里是普通 ``QHBoxLayout``，三列里只有右栏**内部**那条竖分割器
  能拖，现场想"表单拖窄、图拖宽"时无处可拖（S6e）。两段的宽度下限显式钉住，
  比例同样存 ``QSettings``（键 ``body_splitter/<page>``）；
* :func:`build_action_row`：主操作按钮的固定行（不滚动、不被压缩）——
  S6b 把按钮放在预览卡里，卡片被压矮后按钮就跟着滚出/被切半截；
* :class:`MetricGrid` / :class:`StatsGrid`：预览数值与统计卡按需换行
  （宽度不够就换行，既不切字也不白占一行）。
* :func:`mark_result_table`：结果表统一 ``minimumHeight``（表头 + 5 行）；
* :func:`table_key` / :func:`find_row` / :func:`gp_label`：表格第一列与图上
  ``customdata[0]`` 用的是同一个键（生成/修复 = GP 号，ISO = 点名 P1..P5），
  两侧互查都靠这三个函数，避免各页各写一套字符串比较。
* :func:`restore_window_geometry` / :func:`save_window_geometry`：默认窗口尺寸
  与"上次关窗时多大"（``QSettings``）；
* :func:`empty_figure`：切到还没出过图的页时推的空图（清掉上一页的残留）。

S6 第一版把窗口钉成 1180×780 来凑"一屏 5 行"，代价是左侧表单被横向裁掉、
图区只剩 120 px（现场截图里对照很明显）。这个硬指标是定错了：图是这个工具的
核心产物，不该最先被牺牲。所以默认窗口改成 :data:`DEFAULT_WINDOW_SIZE`，
最小 :data:`MINIMUM_WINDOW_SIZE`，缩到最小时允许纵向滚动，但不许横向裁控件。
"""
from __future__ import annotations

from .qt_compat import Qt, QtCore, QtWidgets

#: ``QSettings`` 的组织名 / 应用名（打包后与 ``paths.APP_ID`` 同源，这里是
#: 源码运行时的固定值；换掉会让老用户的布局设置失效）。
SETTINGS_ORG = "robot-calibration"
SETTINGS_APP = "laser-desktop"

#: 默认窗口尺寸（px）：三段都能舒展开，表能露出 8 行。
DEFAULT_WINDOW_SIZE = (1600, 950)

#: 允许缩到的最小窗口（px）：再小就只能横向裁控件了，所以钉死。
MINIMUM_WINDOW_SIZE = (1280, 800)

#: 左侧导航宽度（px）：刚好放下「生成 / 奇异修复 / ISO 五点 / 工具」四项。
#: 原来是 130——导航每多 40 px 都是从表单和图上硬拿的。
NAV_WIDTH = 92

#: ``QSettings`` 里存窗口几何的键（尺寸 + 位置一起存，见 ``saveGeometry``）。
GEOMETRY_KEY = "window/geometry"

#: 结果表的最小高度（px）：表头 + 5 行左右。低于这个数，一屏只能看到表头。
RESULT_TABLE_MIN_HEIGHT = 200

#: 结果区（右栏第三段）至少占的高度（px）：表 200 + 页签条 + 统计 + 状态/进度/
#: 导出。低于它，表就被页签裁掉（**表控件**自己的高度仍是 200，最容易被骗）。
RESULT_MIN_HEIGHT = 300


#: 结果页签的最小高度（px）说明：``QTabWidget`` 的 ``minimumSizeHint`` 只算
#: **当前页**（Qt 的行为）。不显式定高的话，结果区的最小高度会随"当前点开哪个
#: 页签"变：切到「计划」时页签要比它已分配的高度高，Qt 就把表裁掉——实测表自己
#: 还是 200 高，屏幕上只剩 3 行（最容易被"表高够了"骗过去）。所以每页在建完
#: 页签后给一个固定的最小高度（见各页的 ``TABS_MIN_HEIGHT``）。

#: 图区在**默认窗口**下必须达到的高度（px）：3D 图是这个工具的核心产物，
#: 120 px 只能看到一条缝（S6 第一版就是这么退化的）。
PLOT_MIN_HEIGHT = 260

#: 窗口缩到最小时允许的**硬下限**（px）。三段（预览 100 + 图 200 + 结果 ~440）
#: 的最小高度之和必须装得下 1280×800，否则 QMainWindow 会自己变大、或者把
#: 结果表裁掉。所以图控件自己的 ``minimumHeight`` 用这个数，260 由比例保证。
PLOT_FLOOR_HEIGHT = 200

#: 预览卡（外面套一层滚动区）的**硬下限**（px）：窗口再小也留这么高。
#:
#: S6 第一版这里是 50——那不是"最小高度"，是"最容易被裁"：现场截图里卡片只
#: 剩一条标题，按钮和数值全在滚动区下面。S6c 起两件事分开：**首开高度**按内容
#: 算（:func:`preview_default_height`，150–220），**能被挤到多低**由这个数兜底
#: （100 ≈ 标题 + 两行数值）。1280×800 下预览卡会被挤到这个数，这时允许滚动。
PREVIEW_MIN_HEIGHT = 100

#: 预览卡首开高度的**下限**（px）：内容再少也留这么高，卡片不至于像一条缝。
PREVIEW_DEFAULT_MIN = 150

#: 预览卡首开高度的**上限**（px）：内容再多也只给这么高，剩下的靠滚动条。
#: 220 是 1600×950 下"不挤图、不挤表"的上限（图 260 + 结果 440 已经占掉 700）。
PREVIEW_DEFAULT_MAX = 220

#: 预览卡内容高度的余量（px）：刚好等于内容时滚动条会因 1–2 px 的
#: 行距/边框冒出来，留 2 px 才真的“不用滚”。
PREVIEW_KEEP_MARGIN = 2

#: 图 / 结果两段的首开比例（预览卡先按内容拿走自己那一份，剩下的按这个分）。
PLOT_RESULT_RATIOS = (45, 55)


#: 生成 / 修复两页预览卡里的五项数值（同一份，别两边各写一套）：
#: ``设计条件数`` 是判断"可行 / 不可行"的依据，必须在首屏就能看到。
PREVIEW_METRICS = (("volume", "体积"), ("grid", "格点 X×Y×Z"),
                   ("points", "总点数"), ("design", "设计条件数"),
                   ("euler", "欧拉约定"))

#: ``QWidget`` 的默认最大尺寸（``setMaximumHeight`` 用它等于"不设上限"）。
#: 旧代码把主操作按钮压到 24 px，于是 ``height()`` 永远小于 ``sizeHint()``。
QWIDGETSIZE_MAX = (1 << 24) - 1

#: 右栏三段的首开比例（预览卡 / 图 / 结果），合计 100——**折算值**。
#:
#: 预览卡的高度由**内容**决定（:func:`preview_default_height`：内容 sizeHint
#: 夹在 150–220 之间），所以它不是一个真的比例；这里的 16/38/46 是它在
#: 1600×950 下折算出来的份额（150 / 349 / 423 于 922 px 可用高度）：
#:
#: * 预览卡 150 ≥ :data:`PREVIEW_DEFAULT_MIN`，内容无滚动；
#: * 图区 349 → 出结果后由 ``expand_result_area`` 让到 272，仍 ≥
#:   :data:`PLOT_MIN_HEIGHT`（260）；
#: * 结果区 423 ≥ :data:`RESULT_MIN_HEIGHT`（300），撑到 500 后 8 行全见。
#:
#: 用户拖过分割条之后按 ``QSettings`` 里存的**真实**三段比例走，不再按内容折算。
DEFAULT_SPLIT_RATIOS = (16, 38, 46)

#: 比例存进 ``QSettings`` 时的放大倍数（存整数，避免浮点格式差异）。
RATIO_SCALE = 1000

#: 左右两栏（表单 / 右栏）分割条的**手柄宽度**（px）。6 px 在浅色背景上几乎
#: 看不出来，现场反馈"不知道这里能拖"；8 px 才摸得到、也拖得动。
BODY_HANDLE_WIDTH = 8

#: 表单列 / 右栏的宽度**兜底**（px）：真正生效的下限是
#: ``max(兜底, 内容 minimumSizeHint().width())``（见
#: :meth:`BodySplitter.segment_minimums`）。兜底只是防"某页内容特别窄时左栏
#: 能被拖到只剩一条"，三页实测都远大于它：表单 467 / 586 / 524，右栏
#: 301 / 396 / 218（结果表的硬下限）。
BODY_LEFT_FLOOR_WIDTH = 320
BODY_RIGHT_FLOOR_WIDTH = 280

#: 左右两栏的首开比例（千分比，合计 :data:`RATIO_SCALE`）。
#:
#: 口径：1600 宽的窗口里，导航 92 + 页边距 16 + 手柄 8 之后分割器自身 1324 px。
#: 左栏保底 **~540 px**（= 窗口宽的 34%，六个数值框加标签才排得开），
#: 540 / 1324 ≈ **41%**；右栏拿剩下的 59%（≈ 776 px）。
#:
#: 为什么不是直接写 34%：34% 是按**窗口宽**折算的，分割器实际只有 1324 px，
#: 34% 只有 447 px——比表单自己的最小宽度（467）还窄。41% 才真的把 540 px
#: 给到表单。1280 宽的窗口里 41% 只有 408 px，会被抬到各自的
#: ``minimumSizeHint``。
#:
#: 首开还取 **max(这个比例, 表单自身的 ``sizeHint``)**：表单里六组控件本来就是按
#: 自然宽度排的（生成/ISO 560、修复 640），比它窄会把 ``QFormLayout`` 的列挤到
#: 换行。结果就是 1600×950 / 1280×800 下首开与改造前的 ``QHBoxLayout`` **逐像素
#: 一致**（560/640/560），拖动的空间留给用户；比例在窗口比 1600 更宽时才真正
#: 接管。
DEFAULT_BODY_RATIOS = (41, 59)


def default_settings():
    """应用的 ``QSettings``（显式给组织名/应用名，不依赖进程名）。

    不显式给名字的话，``QSettings()`` 会按可执行文件名落盘——源码跑是
    ``python.conf``、打包后是另一个名字，布局设置跟着"换程序"丢一次。
    """
    return QtCore.QSettings(SETTINGS_ORG, SETTINGS_APP)


def restore_window_geometry(window, settings=None) -> bool:
    """把窗口摆回上次关窗时的尺寸/位置（没有记录时什么都不做）。

    Args:
        window: 主窗口（``QMainWindow``）。
        settings: ``QSettings``；``None`` 时用 :func:`default_settings`。

    Returns:
        bool: 读到了可用的记录并恢复成功才 ``True``。
    """
    settings = default_settings() if settings is None else settings
    raw = settings.value(GEOMETRY_KEY)
    if raw is None:
        return False
    if isinstance(raw, QtCore.QByteArray):
        geometry = raw
    elif isinstance(raw, (bytes, bytearray)):
        geometry = QtCore.QByteArray(bytes(raw))
    else:                                       # 坏配置（文本/数字）：忽略
        return False
    if geometry.isEmpty():
        return False
    if not window.restoreGeometry(geometry):    # pragma: no cover - 坏几何
        return False
    # 旧版本存过 1180×780，或者换了更小的显示器：比最小尺寸还小就回默认值
    width, height = window.width(), window.height()
    if (width < MINIMUM_WINDOW_SIZE[0]
            or height < MINIMUM_WINDOW_SIZE[1]):
        window.resize(*DEFAULT_WINDOW_SIZE)
    return True


def save_window_geometry(window, settings=None) -> None:
    """记住窗口的尺寸与位置（关窗时调）。"""
    settings = default_settings() if settings is None else settings
    settings.setValue(GEOMETRY_KEY, window.saveGeometry())
    settings.sync()


def mark_result_table(table) -> None:
    """给结果表定下最小高度（表头 + 5 行），返回同一个表方便链式调用。"""
    table.setMinimumHeight(RESULT_TABLE_MIN_HEIGHT)
    return table


#: 「一张图都没有」时推给壳页的那张空图（同一个对象，主窗口才能按身份去重）。
_EMPTY_FIGURE = None


def empty_figure():
    """一张``data == []``的空图：切到还没出过图的页时用它清掉上一页的图。

    为什么必须是“真推一张空图”而不是“什么都不推”：不推的话上一页的图会留在
    共享 PlotView 上，现场会以为当前页也有结果。壳页见到 ``data == []`` 会
    ``Plotly.purge`` 掉画布并重新露出「还没有数据」（见 ``plot_view`` 的壳页）。

    同一个 Figure 反复用：主窗口按**对象身份**判断“这张图推过了”，每次新建
    一个空对象就永远命不中。

    Returns:
        plotly ``Figure``：没有 trace 的单例。
    """
    global _EMPTY_FIGURE
    if _EMPTY_FIGURE is None:
        import plotly.graph_objects as go

        _EMPTY_FIGURE = go.Figure()
    return _EMPTY_FIGURE


class HintLabel(QtWidgets.QLabel):
    """说明性小字：高度按**当前宽度下文字真正占几行**算（不超过 ``max_lines``）。

    为什么不能固定两行：这类标签大多只有一句话，固定成两行高度后每行都多出
    半行空白——「机器人模型」组里连着两条，看上去就是"行距被拉散、排版坏了"
    （S6b 现场截图）。完全按自然高度撑又会被长文案顶高窗口，所以给一个
    ``max_lines`` 上限，超出部分放 ``tooltip``（与 :func:`fit_message` 同口径）。

    宽度一变（窗口缩放 / 表单列变窄）就重算：``QFormLayout`` 不支持
    ``heightForWidth``，只能自己把高度钉准。
    """

    def __init__(self, text: str = "", max_lines: int = 2, parent=None) -> None:
        super().__init__(text, parent)
        self.max_lines = max(1, int(max_lines))
        self.setWordWrap(True)
        # Fixed：高度就是算出来的自然高度（不再让布局多给一行）
        self.setSizePolicy(QtWidgets.QSizePolicy.Preferred,
                           QtWidgets.QSizePolicy.Fixed)
        self._refit()

    # -------------------------------------------------------------- 高度
    def _line_height(self) -> int:
        return int(self.fontMetrics().lineSpacing())

    def height_for_width(self, width: int) -> int:
        """宽度 ``width`` 时这段文字要几行（上限 :attr:`max_lines` 行）。"""
        line = self._line_height()
        cap = line * self.max_lines
        if int(width) <= 0:                     # 还没有宽度：先按上限占位
            return cap
        rect = self.fontMetrics().boundingRect(
            QtCore.QRect(0, 0, max(1, int(width) - 4), 100000),
            Qt.TextWordWrap, self.text() or " ")
        return min(cap, max(line, int(rect.height()) + 2))

    def _refit(self) -> None:
        height = self.height_for_width(self.width())
        if height != self.height():
            self.setFixedHeight(height)

    def resizeEvent(self, event) -> None:        # noqa: N802 - Qt 回调命名
        super().resizeEvent(event)
        self._refit()

    def setText(self, text) -> None:            # noqa: N802 - Qt 回调命名
        super().setText(text)
        self._refit()


class PlotSlot(QtWidgets.QWidget):
    """页面里的“图位置”：默认一句说明，主窗口把共享的那张图搬进来。

    三页共用**一个** ``PlotView``（计划 §6：切页 < 200 ms、子进程 ≤ 2）。旧写法
    是三页各自把同一个控件 ``addWidget`` 到自己的分割器——Qt 会把它从上一页
    悄悄移走，最后只有最后建的那页能看见图（S6 实测：生成页的分割器里只有
    一个孩子）。所以改成：每页只留一个占位容器，切页时由主窗口把图
    :meth:`adopt` 到当前页。
    """

    def __init__(self, caption: str = "路点图在这里（本环境未挂 WebEngine）",
                 parent=None) -> None:
        super().__init__(parent)
        self._caption = QtWidgets.QLabel(caption)
        self._caption.setAlignment(Qt.AlignCenter)
        self._caption.setWordWrap(True)
        self._caption.setStyleSheet("color:#888")
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self._caption)
        self._widget = None
        # 硬下限用 PLOT_FLOOR_HEIGHT：1280×800 的窗口里，三段最小高度之和必须
        # 装得下（结果表 5 行是硬要求），260 由首开比例保证。
        self.setMinimumHeight(PLOT_FLOOR_HEIGHT)

    def adopt(self, widget) -> None:
        """把共享的图搬进本页（会把它从上一页自动移走）。"""
        if widget is None:
            return
        if self._widget is widget and widget.parent() is self:
            return
        self.layout().addWidget(widget)
        widget.show()
        self._widget = widget
        self._caption.hide()

    @property
    def widget(self):
        """当前放在这里的控件（没搬进来时为 ``None``）。"""
        return self._widget


def gp_label(key) -> str:
    """键 -> 给人看的标签：``5`` -> ``GP0005``；``"P1"`` -> ``P1``。"""
    text = str(key or "").strip()
    if not text:
        return "—"
    try:
        return "GP%04d" % int(float(text))
    except (TypeError, ValueError):
        return text


def normalize_key(value) -> str:
    """把图上传来的键与表格里第一列的文本归一成同一种写法。

    ``customdata[0]`` 是数字（GP）时 JS 侧传回 ``"5"``/``5``/``5.0`` 都可能；
    ISO 的点名是 ``"P1"``。表格第一列写的是 ``"5"`` / ``"P1"``。
    """
    text = str(value if value is not None else "").strip()
    if text[:2].upper() == "GP":
        text = text[2:].strip()
    try:
        return str(int(float(text)))
    except (TypeError, ValueError):
        return text


def table_key(table, row: int):
    """表格第 ``row`` 行第一列的文字（没有这一行/这一格时返回 ``""``）。"""
    if row is None or row < 0 or row >= table.rowCount():
        return ""
    item = table.item(int(row), 0)
    return "" if item is None else str(item.text()).strip()


def find_row(table, key) -> int:
    """按第一列找行号（找不到返回 ``-1``）；三页的表格都可排序，不能按序号找。"""
    wanted = normalize_key(key)
    for row in range(table.rowCount()):
        if normalize_key(table_key(table, row)) == wanted:
            return row
    return -1


def fit_message(label, text, max_height: int = 96) -> None:
    """设置一条会换行的提示文字，并把它的高度钉在**自然高度**（上限 ``max_height``）。

    这类标签（结果区状态行 / 运行目录 / 过期提示）是"一句状态"，但它们的文案
    里带路径与报错原文，一换行就是五六行：按自然高度撑，窗口最小高度会被顶到
    780 以上（实测 900）；改成 ``Ignored`` 又会贪心地把表格挤到只剩 1 行
    （实测）。所以直接把高度钉成**按当前宽度算出的自然高度**（最多
    ``max_height``，约 4 行）：有空间刚好整段显示，超长的那部分靠 ``tooltip``。

    Args:
        label: 要设置的 ``QLabel``（应已 ``setWordWrap(True)``）。
        text: 文字。
        max_height: 高度上限（px），默认 4 行左右。
    """
    text = "" if text is None else str(text)
    label.setText(text)
    label.setToolTip(text)
    label.setWordWrap(True)          # 换行保留：不然最小宽度就是整条路径的长度
    width = max(240, int(label.width() or 0))
    metrics = label.fontMetrics()
    rect = metrics.boundingRect(QtCore.QRect(0, 0, width, 10000),
                                Qt.TextWordWrap, text)
    # Fixed：高度**就是**算出来的自然高度（上限 max_height）。这样它既不肯
    # 定把窗口顶高（max_height 有界），也不会贪心地从表格手里抢高度
    # （``Ignored`` 会抢：实测表格只剩 1 行，虽然表高仍是 200）。
    # 一行高时正好留「一行行距」：多 1 px 都会把第二行切一半挂在下面
    # （截图里像文字叠字）。``.text()`` 不受影响，测试照旧能断言全文。
    if int(max_height) <= 28:
        height = int(metrics.lineSpacing())
    else:
        height = min(int(max_height), int(rect.height()) + 8)
    label.setFixedHeight(height)


class FlowRow(QtWidgets.QWidget):
    """一排「卡」按需换行：每张卡按自己的自然宽度排，装不下就换到下一行。

    为什么不用 ``QGridLayout``：这些卡片（统计卡 7 张、预览数值 5 对）的文字
    长度差好几倍，固定列数要么留一大片空白、要么把最宽的一张挤到切字。
    按需换行既能省高度，又不会横向裁掉任何一张。

    子类给出 :meth:`cell_widgets`（每组是一条不许拆开的卡）与卡内间距；本类
    只管几何：每行从左往右摆，超出宽度就换行，高度用 ``setFixedHeight`` 钉成
    实际行数（只在行数/高度变了时重设，否则会把父布局抽死）。
    """

    #: 一张卡的高度（px）。
    ITEM_HEIGHT = 22

    #: 卡与卡之间的水平间距（px）。
    H_SPACING = 12

    #: 行与行之间的垂直间距（px）。
    V_SPACING = 2

    #: 一张卡内部（名称/数值）的间距（px）。
    CELL_GAP = 6

    def __init__(self, parent=None) -> None:
        """建空排（子类负责把控件建出来）。"""
        super().__init__(parent)
        self._rows = 0
        self.setSizePolicy(QtWidgets.QSizePolicy.Preferred,
                           QtWidgets.QSizePolicy.Fixed)

    # ------------------------------------------------------------ 子类接口
    def cell_widgets(self):
        """返回 ``[[控件, ...], ...]``：每个子序列是一条不许拆开的卡。"""
        raise NotImplementedError

    @property
    def rows(self) -> int:
        """当前排了几行（测试/冒烟要看这个数字）。"""
        return self._rows

    # ------------------------------------------------------------ 几何
    def cell_width(self, widgets) -> int:
        """一张卡的自然宽度（px）。"""
        width = sum(max(1, widget.sizeHint().width()) for widget in widgets)
        return width + self.CELL_GAP * max(0, len(widgets) - 1)

    def sizeHint(self) -> QtCore.QSize:          # noqa: N802 - Qt 覆写
        """理想尺寸：所有卡排一行（真实高度看 :meth:`height_for_width`）。"""
        cells = self.cell_widgets()
        width = sum(self.cell_width(cell) for cell in cells)
        width += self.H_SPACING * max(0, len(cells) - 1)
        return QtCore.QSize(max(1, width), self.ITEM_HEIGHT)

    def height_for_width(self, width: int) -> int:
        """宽度 ``width`` 时这些卡排完一共多高（px；不改变控件几何）。

        为什么要有它：``sizeHint()`` 的高度恒等于一行（不分宽窄），而卡片外面
        的 ``QFormLayout``/``QVBoxLayout`` 只按 ``sizeHint`` 求和算"内容多高"。
        窗口一窄、卡换行，算出来的高度就比真的矮一截（S6 评审建议 6：1280×800
        下 ISO 预览卡实际 181 px，按 ``sizeHint`` 只有 156 px，卡片就带着竖滚动
        条），:func:`preview_default_height` / :func:`preview_keep_height` 用这个
        函数把差额补上。

        Args:
            width: 可用宽度（px）；<=0 时按一行算。

        Returns:
            int: 高度（px）；没有卡时为 0。
        """
        cells = self.cell_widgets()
        if not cells:
            return 0
        width = int(width)
        if width <= 0:
            return self.ITEM_HEIGHT
        x = y = 0
        row_height = 0
        for widgets in cells:
            cell_w = self.cell_width(widgets)
            if x > 0 and x + cell_w > width:
                x = 0
                y += row_height + self.V_SPACING
                row_height = 0
            x += cell_w + self.H_SPACING
            row_height = max(row_height, self.ITEM_HEIGHT)
        return y + row_height

    def minimumSizeHint(self) -> QtCore.QSize:   # noqa: N802 - Qt 覆写
        """最小尺寸：最宽的一张卡 + 当前行数的高度（高度由 :meth:`relayout` 钉）。"""
        cells = self.cell_widgets()
        widest = max([1] + [self.cell_width(cell) for cell in cells])
        return QtCore.QSize(widest, max(self.ITEM_HEIGHT, self.height()))

    def relayout(self) -> int:
        """按当前宽度重排，返回行数（宽度为 0 时先按一行占位）。"""
        cells = self.cell_widgets()
        if not cells:
            return 0
        width = int(self.width())
        if width <= 0:
            width = max(1, self.sizeHint().width())
        x = y = 0
        row_height = 0
        rows = 1
        for widgets in cells:
            cell_w = self.cell_width(widgets)
            if x > 0 and x + cell_w > width:
                x = 0
                y += row_height + self.V_SPACING
                row_height = 0
                rows += 1
            offset = x
            for widget in widgets:
                item_w = max(1, widget.sizeHint().width())
                widget.setGeometry(offset, y, item_w, self.ITEM_HEIGHT)
                offset += item_w + self.CELL_GAP
            x += cell_w + self.H_SPACING
            row_height = max(row_height, self.ITEM_HEIGHT)
        height = y + row_height
        if rows != self._rows or height != self.height():
            self._rows = rows
            self.setFixedHeight(height)
        return rows

    def resizeEvent(self, event) -> None:        # noqa: N802 - Qt 回调命名
        """宽度变了：重排（只在行数/高度变了时重新定高）。"""
        super().resizeEvent(event)
        self.relayout()


class StatsGrid(FlowRow):
    """结果区的统计卡：一行能放几张放几张。

    七张卡宽度差 3 倍（"可行" 34 px、"design 14.365 / post_ik —" 237 px）：
    固定 3 列时第三行只剩 "可行" 一张，白占 24 px 高；固定 4 列又会把最宽的
    那张挤到切字。实测 1600×950（右栏 ~738 px）两行排完、1280×800 三行。

    那 24 px 在 1600×950 下正好是修复页第 8 行 / 预览卡放得下内容的那一截
    （真实窗口分割器只有 ~892 px）。
    """

    def __init__(self, labels: dict, keys, parent=None) -> None:
        """按 ``keys`` 顺序建标签（文字由 :func:`fill_stat_labels` 填）。"""
        super().__init__(parent)
        self.labels = labels
        self.keys = tuple(keys)
        for key in self.keys:
            label = QtWidgets.QLabel("—", self)
            label.setTextInteractionFlags(Qt.TextSelectableByMouse)
            labels[key] = label
        self.relayout()

    def cell_widgets(self):
        """一卡一张标签。"""
        return [[self.labels[key]] for key in self.keys]


class MetricGrid(FlowRow):
    """预览卡的「名称 数值」对：一行能放几对放几对。

    S6b 是五行 ``QFormLayout``（每行一对，141 px）；S6c 改成按需换行：1600×950
    下两行（~70 px）、1280×800 下两三行——「设计条件数」不再被挤到滚动区外面。
    每对内部是"名称 + 数值"同一行，卡与卡之间留 16 px。
    """

    #: 一对的高度（px）：比统计卡矮一点（卡片里没有状态色块）。
    ITEM_HEIGHT = 21

    #: 对与对之间的水平间距（px）：比统计卡宽一点，肉眼能分组。
    H_SPACING = 16

    def __init__(self, labels: dict, keys, parent=None) -> None:
        """按 ``(键, 名称)`` 建名称/数值标签。"""
        super().__init__(parent)
        self.labels = labels
        self.keys = tuple(keys)
        self.names = []
        for key, title in self.keys:
            name = QtWidgets.QLabel(title, self)
            name.setStyleSheet("color:#6b7280")
            value = QtWidgets.QLabel("—", self)
            value.setTextInteractionFlags(Qt.TextSelectableByMouse)
            labels[key] = value
            self.names.append(name)
        self.relayout()

    def cell_widgets(self):
        """一卡 = 名称 + 数值（同一行，不许拆开）。"""
        return [[name, self.labels[key]]
                for name, (key, _title) in zip(self.names, self.keys)]


def build_action_row(*buttons) -> QtWidgets.QWidget:
    """把主操作按钮装进一行**不滚动、不被压缩**的操作行，返回那个 ``QWidget``。

    为什么必须独立出来：S6b 把按钮放在预览卡**里面**，卡片被压到 50 px 后
    按钮就跟着滚出去 / 被上边缘切掉半截（现场截图：修复页「定位段」「修复」
    只剩下半截）。这里的三件事保证它永远完整：

    * 操作行在**滚动区之外**（挂在左栏底部，和表单滚动区平级）；
    * 垂直策略 ``Fixed``：布局按 ``sizeHint`` 给，不参压缩；
    * 按钮不设 ``setMaximumHeight``：``height()`` 才能 ≥ ``sizeHint()``。

    Args:
        *buttons: 要摆进这一行的按钮（按顺序靠左，右侧自动留白）。

    Returns:
        QtWidgets.QWidget: 操作行（``objectName = "action_row"``）。
    """
    row = QtWidgets.QWidget()
    row.setObjectName("action_row")
    layout = QtWidgets.QHBoxLayout(row)
    layout.setContentsMargins(0, 0, 0, 0)
    layout.setSpacing(6)
    for button in buttons:
        button.setMaximumHeight(QWIDGETSIZE_MAX)      # 不压扁（旧代码压到 24）
        button.setMinimumHeight(button.sizeHint().height())
        layout.addWidget(button)
    layout.addStretch(1)
    row.setSizePolicy(QtWidgets.QSizePolicy.Preferred,
                      QtWidgets.QSizePolicy.Fixed)
    row.setFixedHeight(row.sizeHint().height())
    return row


def flow_height_extra(widget) -> int:
    """``widget`` 里的卡在**当前宽度**下比现在"以为的"多占的高度（px）。

    ``FlowRow.sizeHint()`` 的高度恒等于一行（:data:`FlowRow.ITEM_HEIGHT`），而
    卡片外层的布局只按 ``sizeHint``/``minimumSizeHint`` 求和——窗口一变窄、
    数值卡还没重新排完时，算出来的"内容高度"就比真的矮（S6 评审建议 6：
    1280×800 下 ISO 预览卡实际要 181 px，按旧口径只有 156 px）。

    基准取 ``max(sizeHint().height(), height())``——即布局此刻真的给它算的那
    一份（``QWidgetItem::sizeHint`` 就是 ``sizeHint`` expandedTo
    ``minimumSizeHint``）；已经按当前宽度换好行的卡因此不会被重复加一遍。

    Args:
        widget: 卡片（``QGroupBox``）或任何包含 :class:`FlowRow` 的控件。

    Returns:
        int: 补偿高度（>= 0；控件已被删、没有宽度或已排好时为 0）。
    """
    try:
        flows = widget.findChildren(FlowRow)
    except (AttributeError, RuntimeError):       # 已被删 / 不是 QWidget
        return 0
    extra = 0
    for flow in flows:
        try:
            base = max(flow.sizeHint().height(), flow.height())
            actual = flow.height_for_width(flow.width())
        except RuntimeError:                     # 子控件已被删
            continue
        extra += max(0, int(actual) - int(base))
    return extra


def preview_default_height(widget=None) -> int:
    """预览卡的首开高度：内容 ``sizeHint`` 夹在 [150, 220] 之间。

    S6b 之前预览卡的高度来自一个固定比例（25%），卡片自己再套一层滚动区、
    最小高度写 50——于是窗口一变，先被牺牲的就是它。现在反过来：**内容多高
    就多高**（上限 220，超出部分滚动），只有窗口真的装不下时才会被压到
    :func:`preview_keep_height`（1280×800 / 出结果后的 1600×950 就是这种情况）。

    "内容高度"要按**当前宽度**算：窄窗口下数值卡换行，``sizeHint`` 会低估
    （见 :func:`flow_height_extra`）。

    Args:
        widget: 预览卡（``QGroupBox``）；``None`` 时返回下限。

    Returns:
        int: 高度（px）。
    """
    hint = 0
    if widget is not None:
        try:
            hint = int(widget.sizeHint().height()) + flow_height_extra(widget)
        except RuntimeError:            # 控件已被删
            hint = 0
    return max(PREVIEW_DEFAULT_MIN, min(PREVIEW_DEFAULT_MAX, hint))


def preview_keep_height(widget=None) -> int:
    """出结果后预览卡可以降到的下限：**放得下内容**（不超 150，不低于 100）。

    为什么不是一个常数：1600×950 的真实窗口里，分割器只有 ~892 px，而
    "8 行表 + 图 260" 要 765–772 px——剩下的 120–127 px 就是预览卡的全部。
    卡片内容本身只有 115 px（五项数值 + 标题），所以压到 115 仍然**一眼全见**；
    而内容更高的卡片（比如 200 px）不该被压到 115，那样就藏东西了，所以
    上限卡在 :data:`PREVIEW_DEFAULT_MIN`。

    Args:
        widget: 预览卡（``QGroupBox``）；``None`` 时返回 :data:`PREVIEW_MIN_HEIGHT`。

    Returns:
        int: 高度（px）。
    """
    hint = 0
    if widget is not None:
        try:
            hint = (int(widget.minimumSizeHint().height())
                    + flow_height_extra(widget))
        except RuntimeError:            # 控件已被删
            hint = 0
    if hint <= 0:
        return PREVIEW_MIN_HEIGHT
    return max(PREVIEW_MIN_HEIGHT,
               min(PREVIEW_DEFAULT_MIN, hint + PREVIEW_KEEP_MARGIN))


class RightSplitter(QtWidgets.QSplitter):
    """右栏三段：预览卡 / 图 / 结果区，比例可拖、可记忆。

    Attributes:
        name: 页面名（``"generate"`` / ``"repair"`` / ``"iso"``），也是
            ``QSettings`` 的键的一部分。
        ratios: 当前比例（千分比，合计 :data:`RATIO_SCALE`）。
    """

    def __init__(self, name: str, preview, viewer, results, settings=None,
                 ratios=DEFAULT_SPLIT_RATIOS) -> None:
        """建三段分割器。

        Args:
            name: 页面名（QSettings 键 ``right_splitter/<name>``）。
            preview: 预览卡片。
            viewer: 图（``PlotView`` / 懒创建容器 / 无图环境下的说明标签）。
            results: 结果区。
            settings: ``QSettings`` 实例；``None`` 时用 :func:`default_settings`。
            ratios: 首开比例（没有存过设置时用）。
        """
        super().__init__(Qt.Vertical)
        self.name = str(name)
        self._settings = settings
        self._applying = False
        self._applied = False
        self._protecting = False
        self._user_moved = False
        self._preview_card = preview
        for widget in (_wrap_scrollable(preview, PREVIEW_MIN_HEIGHT), viewer,
                       results):
            self.addWidget(widget)
            self.setStretchFactor(self.count() - 1, 1)
        self.setChildrenCollapsible(False)
        self.setHandleWidth(6)
        stored = self.saved_ratios()
        self._from_settings = stored is not None
        self._ratios = stored if stored is not None else _normalize(ratios)
        self.splitterMoved.connect(self._on_moved)

    @property
    def preview_card(self):
        """预览卡本体（不是外面那层滚动区）——量内容高度用。"""
        return self._preview_card

    @property
    def user_moved(self) -> bool:
        """用户亲手拖过分割条没有（拖过就不许程序再把结果区缩回去）。"""
        return bool(self._user_moved)

    # ------------------------------------------------------------ 设置读写
    def settings_key(self) -> str:
        """``QSettings`` 里的键名（一个页面一个）。"""
        return f"right_splitter/{self.name}"

    def settings(self):
        """实际的 ``QSettings``（测试注入时就是注入的那个）。"""
        if self._settings is None:
            self._settings = default_settings()
        return self._settings

    def saved_ratios(self):
        """读回上次存的比例（千分比三元组）；没存过/读坏了返回 ``None``。"""
        try:
            raw = self.settings().value(self.settings_key())
        except (TypeError, ValueError):             # pragma: no cover - 坏配置
            return None
        return _parse_ratios(raw)

    def save_ratios(self) -> None:
        """把**当前**比例写进 ``QSettings``（用户拖动分割条后自动调）。"""
        self._ratios = self.live_ratios()
        self.settings().setValue(self.settings_key(), list(self._ratios))

    def reset_ratios(self) -> None:
        """回到首开比例（冒烟脚本/测试要可复现的截图时用）。"""
        self._ratios = _normalize(DEFAULT_SPLIT_RATIOS)
        self._from_settings = False
        self._applied = False
        self.apply_ratios()

    # ------------------------------------------------------------ 比例
    @property
    def ratios(self):
        """下一轮布局要用的比例（千分比）：存过就是存的，没存过就是默认值。

        注意这是**设计比例**，不是屏幕上现在的像素比例：结果区会按内容把自己
        撑到够看 8 行（:func:`laser_desktop.generate_page.expand_result_area`），
        屏幕上的真实比例看 :meth:`live_ratios`。
        """
        return tuple(self._ratios)

    def live_ratios(self):
        """屏幕上**现在的**比例（千分比）；还没布局过时返回设计比例。"""
        sizes = self.sizes()
        total = sum(sizes)
        if total <= 0 or len(sizes) != 3:
            return tuple(self._ratios)
        return _normalize([size * RATIO_SCALE / total for size in sizes])

    def _children_min_height(self) -> int:
        """三段的最小高度之和（含分隔条）：低于它就别谈"按比例分配"了。"""
        total = 0
        for index in range(self.count()):
            widget = self.widget(index)
            if widget is None:
                continue
            total += max(int(widget.minimumHeight()),
                         int(widget.minimumSizeHint().height()))
        return total + self.handleWidth() * max(0, self.count() - 1)

    def apply_ratios(self) -> bool:
        """按当前比例分配尺寸；控件还没有高度时返回 ``False``（等 resize）。

        首开（``QSettings`` 里没有记录）时：预览卡先按**内容高度**拿走自己那一
        份（:func:`preview_default_height`，150–220），剩下的按
        :data:`PLOT_RESULT_RATIOS` 分给图 / 结果区。用户拖过之后（存过设置）
        则按存下来的**真实**三段比例走。

        分配完再 :meth:`protect_plot` 从预览卡借：结果区的**内容**最小高度
        （表 200 + 页签 + 统计）常常超过它那份，Qt 会把图挤到只剩一条缝
        （S6 第一版实测 120 px）。图优先，预览卡往后稍。

        当前高度装不下三段的最小高度时**不**分配：``QStackedWidget`` 会给隐藏
        页塞一个临时高度（实测 464），在那里定比例会得到 9.6/27.8/62.5 这种
        结果，并且因为"已经分配过"再也不修正。
        """
        total = self.height()
        if total <= 0:
            return False
        if total < self._children_min_height():
            return False
        self._apply_sizes(self.default_sizes(total))
        self.protect_plot()
        self._applied = True
        return True

    def default_sizes(self, total: int):
        """首开尺寸（三段像素）：预览卡按内容，图 / 结果按 :data:`PLOT_RESULT_RATIOS`。

        用户拖过（``QSettings`` 里有记录）就按存下来的三段比例算。

        Args:
            total: 分割器可用高度（px，含分隔条，口径与 ``sizes()`` 之和相同）。

        Returns:
            list[int]: 三段高度。
        """
        total = int(total)
        usable = max(3, total - self.handleWidth() * max(0, self.count() - 1))
        if self._from_settings:
            sizes = [max(1, int(usable * ratio / RATIO_SCALE))
                     for ratio in self._ratios]
            sizes[-1] = max(1, usable - sizes[0] - sizes[1])
            return sizes
        preview = min(preview_default_height(self._preview_card),
                      max(1, usable - 2))
        rest = max(2, usable - preview)
        plot, result = PLOT_RESULT_RATIOS
        plot_size = max(1, int(rest * plot / (plot + result)))
        return [preview, plot_size, max(1, rest - plot_size)]

    def plot_floor(self) -> int:
        """图区在这一档窗口里能被压到的下限（px）：装得下就 260，装不下就 200。

        1600×950 的真实窗口（分割器 ~892 px）装得下预览 keep + 260 + 结果区
        最小高度，所以图区守 :data:`PLOT_MIN_HEIGHT`；1280×800 装不下（需要
        ~827 px，实际只有 ~742），再守 260 只会把结果表挤到 4 行——那时图区
        按硬下限 :data:`PLOT_FLOOR_HEIGHT` 走。

        用户亲手拖过分割条（:attr:`user_moved`）时直接给硬下限：剩下的由用户
        自己说了算，程序不再拿 260 去量。

        Returns:
            int: 可接受的最低高度。
        """
        if self._user_moved:
            return PLOT_FLOOR_HEIGHT
        results = self.widget(2)
        results_min = RESULT_MIN_HEIGHT
        if results is not None:
            results_min = max(results_min,
                              int(results.minimumSizeHint().height()))
        needed = (preview_keep_height(self._preview_card) + PLOT_MIN_HEIGHT
                  + results_min + 2 * self.handleWidth())
        return PLOT_MIN_HEIGHT if self.height() >= needed else PLOT_FLOOR_HEIGHT

    def protect_plot(self) -> bool:
        """图区矮于 :data:`PLOT_MIN_HEIGHT` 时把高度凑回来；返回是否调过。

        先向**预览卡**借（不低于 :func:`preview_keep_height`，即“放得下内容”），
        不够再从**结果区**的余量里拿（只动它超出自身最小高度的部分）。

        只在**空间真的够**的时候借：三段的**设计**下限（预览 :func:`preview_keep_height`
        + 图 260 + 结果内容最小高度）加起来装不下时（1280×800 就是这样），硬抬
        图区只会把预览卡压穿、再被 Qt 按最小高度弹回去，反而不稳。那种情况下
        图区按自己的硬下限 :data:`PLOT_FLOOR_HEIGHT` 走。

        **用户拖过分割条就不借**（S6 评审 R3）：``RightSplitter.user_moved`` 的
        语义是“拖过就不许程序再把结果区缩回去”，而这里原来不认这个标志——
        实测 1600×950 下置了 ``_user_moved`` 再 ``setSizes([300,150,452])``，图区
        立刻被抬回 260（预览卡被扣 84、结果区被扣 36）。现在与
        :meth:`shrink_result_area` 同一口径：用户说了算。

        Returns:
            bool: 真的调过了才 ``True``。
        """
        if self._user_moved:                        # 用户拖过：不抢高度
            return False
        sizes = list(self.sizes())
        if len(sizes) != 3:
            return False
        short = PLOT_MIN_HEIGHT - sizes[1]
        if short <= 0:
            return False
        results = self.widget(2)
        results_min = RESULT_MIN_HEIGHT
        if results is not None:
            results_min = max(results_min,
                              int(results.minimumSizeHint().height()))
        keep = preview_keep_height(self._preview_card)
        needed = (keep + PLOT_MIN_HEIGHT + results_min
                  + 2 * self.handleWidth())
        if self.height() < needed:                  # 装不下设计下限：不借
            return False
        from_preview = min(short, max(0, sizes[0] - keep))
        from_results = 0
        if from_preview < short and results is not None:
            # 预览卡已到“放得下内容”的下限：剩下的几 px 从**结果区**的余量里拿
            # （只动它超出自身最小高度的部分）。实测修复页：1280×800 把图区压到
            # 211，放大回 1600×950 后按比例只回到 254——差的就是这 6 px，而
            # 结果区那时比它需要的高 10 px。
            spare = max(0, sizes[2] - max(RESULT_MIN_HEIGHT,
                                          int(results.minimumSizeHint().height())))
            from_results = min(short - from_preview, spare)
        give = from_preview + from_results
        if give <= 0:
            return False
        sizes[0] -= from_preview
        sizes[2] -= from_results
        sizes[1] += give
        self._apply_sizes(sizes)
        return True

    def _apply_sizes(self, sizes) -> None:
        """程序性地设尺寸（不当作"用户拖过"写进 QSettings）。"""
        self._applying = True
        try:
            self.setSizes(list(sizes))
        finally:
            self._applying = False

    def _on_moved(self, *_args) -> None:
        """用户拖动分割条：把结果比例存下来（程序自己 setSizes 时不存）。"""
        if self._applying:
            return
        self._user_moved = True
        self.save_ratios()

    def showEvent(self, event) -> None:             # noqa: N802 - Qt 回调命名
        """第一次显示时按比例分配（构造时还没有高度）。"""
        super().showEvent(event)
        if not self._applied:
            self.apply_ratios()

    def event(self, event) -> bool:                 # noqa: N802 - Qt 回调命名
        """子控件的最小高度变了（切页签 / 出结果）就再护一次图区。"""
        handled = super().event(event)
        if event.type() == QtCore.QEvent.LayoutRequest:
            self._protect_after_layout()
        return handled

    def _protect_after_layout(self) -> None:
        """布局重排后护一次图区（带重入护栏：protect 里的 setSizes 也会引发布局）。"""
        if self._protecting:
            return
        self._protecting = True
        try:
            self.protect_plot()
        finally:
            self._protecting = False

    def resizeEvent(self, event) -> None:           # noqa: N802 - Qt 回调命名
        """第一次拿到高度时按比例分配；之后交给 Qt 按当前比例缩放。"""
        super().resizeEvent(event)
        if not self._applied:
            self.apply_ratios()


class BodySplitter(QtWidgets.QSplitter):
    """页面左右两半（表单列 / 右栏）之间的**横向**分割器，可拖、可记忆。

    S6e 之前这里是普通 ``QHBoxLayout``：三列（左导航 / 表单 / 右栏）里只有
    右栏**内部**那条竖分割器能拖，现场想把表单拖窄、把图拖宽时发现无处可拖。
    换成 ``QSplitter`` 之后：

    * 两段的宽度下限显式钉在控件上（:meth:`apply_minimum_widths`：左 = 表单的
      ``minimumSizeHint().width()``，右 = 图与结果表挤出来的硬下限），再加
      ``setChildrenCollapsible(False)``——左边拖到 0、右边拖到 0 都会被挡在下限，
      数值框不会被裁掉；
    * 比例存 ``QSettings``（键 ``body_splitter/<page>``），下次开窗读回来；
      存过的比例在这一档宽度上装不下时回默认比例（与 :class:`RightSplitter`
      同一套“存过但比最小尺寸小就回默认”的判据）；
    * **只管横向**：竖分割器（:class:`RightSplitter`）的 ``protect_plot`` /
      ``plot_floor`` 不认它，用户拖完不会被程序抢回去。程序只在首次布局时
      按比例分一次（:meth:`apply_ratios`），之后完全交给 ``QSplitter``——窗口
      resize 时 Qt 本来就按当前比例缩放。

    Attributes:
        name: 页面名（``"generate"`` / ``"repair"`` / ``"iso"``），也是
            ``QSettings`` 键的一部分。
        ratios: 当前设计比例（千分比二元组，合计 :data:`RATIO_SCALE`）。
    """

    def __init__(self, name: str, left, right, settings=None,
                 ratios=DEFAULT_BODY_RATIOS) -> None:
        """建两段横向分割器（左 = 表单列，右 = 右栏）。

        Args:
            name: 页面名（QSettings 键 ``body_splitter/<name>``）。
            left: 左段（``_build_left_column()`` 的返回物：表单滚动区 + 操作行）。
            right: 右段（``_build_right_column()`` 的返回物：
                :class:`RightSplitter`）。
            settings: ``QSettings`` 实例；``None`` 时用 :func:`default_settings`。
            ratios: 首开比例（没有存过设置时用）。
        """
        super().__init__(Qt.Horizontal)
        self.name = str(name)
        self._settings = settings
        self._applying = False
        self._applied = False
        self._user_moved = False
        #: 存过的比例在当前宽度上会把某段压到硬下限以下（:meth:`default_sizes`
        #: 已经改回默认比例）。测试/冒烟要看这个状态。
        self._stored_too_narrow = False
        self._left = left
        self._right = right
        for widget in (left, right):
            self.addWidget(widget)
            self.setStretchFactor(self.count() - 1, 1)
        self.setChildrenCollapsible(False)
        self.setHandleWidth(BODY_HANDLE_WIDTH)
        self.apply_minimum_widths()
        stored = self.saved_ratios()
        self._from_settings = stored is not None
        self._ratios = (stored if stored is not None
                        else _normalize(ratios, 2, DEFAULT_BODY_RATIOS))
        self.splitterMoved.connect(self._on_moved)

    # ------------------------------------------------------------ 两段
    @property
    def left_column(self):
        """左段控件（表单列）。"""
        return self._left

    @property
    def right_column(self):
        """右段控件（右栏 = 竖分割器）。"""
        return self._right

    @property
    def user_moved(self) -> bool:
        """用户亲手拖过没有（拖过就不再接受程序改宽度）。"""
        return bool(self._user_moved)

    @staticmethod
    def _segment_minimum(widget, floor: int) -> int:
        """一段的硬下限（px）：``max(兜底, minimumSizeHint, 已设的 minimumWidth)``。"""
        try:
            hint = int(widget.minimumSizeHint().width())
            current = int(widget.minimumWidth())
        except RuntimeError:                    # 控件已被删
            hint = current = 0
        return max(int(floor), hint, current)

    def segment_minimums(self):
        """两段的硬下限 ``[左, 右]``（px）。

        左段 = 表单的 ``minimumSizeHint().width()``（三页实测 467 / 586 / 524，
        含纵向滚动条与边框）；右段 = 图与结果表挤出来的宽度（301 / 396 / 218，
        就是 :class:`RightSplitter` 自己的 ``minimumSizeHint().width()``）。
        两者都没到兜底时用 :data:`BODY_LEFT_FLOOR_WIDTH` /
        :data:`BODY_RIGHT_FLOOR_WIDTH`。

        Returns:
            list[int]: ``[左段最小宽, 右段最小宽]``。
        """
        return [self._segment_minimum(self._left, BODY_LEFT_FLOOR_WIDTH),
                self._segment_minimum(self._right, BODY_RIGHT_FLOOR_WIDTH)]

    def apply_minimum_widths(self) -> None:
        """把两段的硬下限钉到控件上（拖到极端也不许裁内容）。

        为什么不用布局自己的 ``minimumSizeHint`` 兜底就够了：``QSplitter`` 确实
        认子控件的 ``minimumSizeHint``，但那是**隐式**的——一旦某段的最小尺寸
        跟着内容变（切页签 / 出结果），生效的下限也跟着变。显式 ``setMinimumWidth``
        让“拖不动到这里”有一条测试量与代码读得懂的线，再让 ``QSplitter`` 在两者
        里取大即可。
        """
        left_min, right_min = self.segment_minimums()
        self._left.setMinimumWidth(left_min)
        self._right.setMinimumWidth(right_min)

    def children_min_width(self) -> int:
        """两段 + 手柄的最小宽度之和（低于它就别谈“按比例分配”了）。"""
        left_min, right_min = self.segment_minimums()
        return (left_min + right_min
                + self.handleWidth() * max(0, self.count() - 1))

    def natural_left_width(self) -> int:
        """表单想要的宽度（左段 ``sizeHint``）：六组控件就是按这个宽度排的。

        生成/ISO 560、修复 640（实测），恰好是改造前 ``QHBoxLayout`` 给左栏的
        宽度（``setStretchFactor(0, 0)`` 就是“拿 sizeHint”）。
        """
        try:
            return max(0, int(self._left.sizeHint().width()))
        except RuntimeError:                     # 控件已被删
            return 0

    # ------------------------------------------------------------ 设置读写
    def settings_key(self) -> str:
        """``QSettings`` 里的键名（一个页面一个）。"""
        return f"body_splitter/{self.name}"

    def settings(self):
        """实际的 ``QSettings``（测试注入时就是注入的那个）。"""
        if self._settings is None:
            self._settings = default_settings()
        return self._settings

    def saved_ratios(self):
        """读回上次存的比例（千分比二元组）；没存过/读坏了返回 ``None``。"""
        try:
            raw = self.settings().value(self.settings_key())
        except (TypeError, ValueError):             # pragma: no cover - 坏配置
            return None
        return _parse_ratios(raw, 2, DEFAULT_BODY_RATIOS)

    def save_ratios(self) -> None:
        """把**当前**比例写进 ``QSettings``（用户拖动分割条后自动调）。"""
        self._ratios = self.live_ratios()
        self.settings().setValue(self.settings_key(), list(self._ratios))

    def reset_ratios(self) -> None:
        """回到首开比例（冒烟脚本/测试要可复现的截图时用）。"""
        self._ratios = _normalize(DEFAULT_BODY_RATIOS, 2, DEFAULT_BODY_RATIOS)
        self._from_settings = False
        self._applied = False
        self.apply_ratios()

    # ------------------------------------------------------------ 比例
    @property
    def ratios(self):
        """下一轮布局要用的比例（千分比二元组）：存过就是存的，否则是默认值。"""
        return tuple(self._ratios)

    def live_ratios(self):
        """屏幕上**现在的**比例（千分比）；还没布局过时返回设计比例。"""
        sizes = self.sizes()
        total = sum(sizes)
        if total <= 0 or len(sizes) != 2:
            return tuple(self._ratios)
        return _normalize([size * RATIO_SCALE / total for size in sizes],
                          2, DEFAULT_BODY_RATIOS)

    def default_sizes(self, total: int):
        """首开尺寸（两段像素宽）：按比例算，再抬到各自的硬下限。

        顺序：先看 ``QSettings`` 里存过的比例——它在这一档宽度上**两段都不低于
        下限**才用；只要有一段被压到下限以下（比如在宽窗口拖出来的比例，在
        1280 窗口里装不下表单），就当作“存过的比最小尺寸还小”，回默认
        （判据与 :meth:`restore_window_geometry` 的“存过但比最小尺寸小就回默认”
        是同一套，见 :attr:`_stored_too_narrow`）。

        首开的默认值取 **max(比例, 表单的 ``sizeHint``)**：比表单自然宽度窄会
        把 ``QFormLayout`` 的列挤到换行，而改造前 ``QHBoxLayout`` 给左栏的就是
        ``sizeHint``——两者一致，改完首开看不出来，想窄自己去拖。

        Args:
            total: 分割器当前宽度（px，含手柄，口径与 ``QSplitter.width()`` 相同）。

        Returns:
            list[int]: 两段宽度，合计 = ``total - handleWidth``。
        """
        total = int(total)
        usable = max(2, total
                     - self.handleWidth() * max(0, self.count() - 1))
        mins = self.segment_minimums()
        self._stored_too_narrow = False
        if self._from_settings:
            wanted = [usable * ratio / RATIO_SCALE for ratio in self._ratios]
            if wanted[0] >= mins[0] and wanted[1] >= mins[1]:
                return _fit_pair(wanted, mins, usable)
            self._stored_too_narrow = True
        wanted_left = max(usable * DEFAULT_BODY_RATIOS[0] / RATIO_SCALE,
                          float(self.natural_left_width()))
        return _fit_pair([wanted_left, usable - wanted_left], mins, usable)

    def apply_ratios(self) -> bool:
        """按当前比例分配宽度；控件还没有宽度/装不下下限时返回 ``False``。

        只在**首次**布局时调（``showEvent`` / ``resizeEvent`` 里 ``_applied``
        为假时）。之后绝不重算：用户拖过的宽度、窗口 resize 后的比例都由
        ``QSplitter`` 自己维持，程序插手就会把用户的拖动“抢回去”。

        Returns:
            bool: 真的分配过才 ``True``。
        """
        total = self.width()
        if total <= 0:
            return False
        if total < self.children_min_width():
            return False
        self._apply_sizes(self.default_sizes(total))
        self._applied = True
        return True

    def _apply_sizes(self, sizes) -> None:
        """程序性地设尺寸（不当作“用户拖过”写进 ``QSettings``）。"""
        self._applying = True
        try:
            self.setSizes(list(sizes))
        finally:
            self._applying = False

    def _on_moved(self, *_args) -> None:
        """用户拖动分割条：把比例存下来（程序自己 setSizes 时不存）。"""
        if self._applying:
            return
        self._user_moved = True
        self.save_ratios()

    def showEvent(self, event) -> None:             # noqa: N802 - Qt 回调命名
        """第一次显示时按比例分配（构造时还没有宽度）。"""
        super().showEvent(event)
        if not self._applied:
            self.apply_ratios()

    def resizeEvent(self, event) -> None:           # noqa: N802 - Qt 回调命名
        """第一次拿到宽度时按比例分配；之后交给 Qt 按当前比例缩放。"""
        super().resizeEvent(event)
        if not self._applied:
            self.apply_ratios()


def _fit_pair(wanted, minimums, usable: int):
    """把两段的设计宽度抬到下限、再凑成 ``usable``（多余给右段、不够从右段扣）。

    Args:
        wanted: 设计宽度（浮点，两段）。
        minimums: 两段的硬下限（px）。
        usable: 两段宽度之和（px，已经扣掉手柄）。

    Returns:
        list[int]: 两段宽度，合计 = ``usable``（下限之和超过 ``usable`` 时只能
        ``QRect`` 式的截断——那种情况 ``apply_ratios`` 根本不会调到本函数）。
    """
    sizes = [max(int(round(float(width))), int(low))
             for width, low in zip(wanted, minimums)]
    excess = sum(sizes) - int(usable)
    for index in (1, 0):                        # 先扣右段（图/结果有余量时）
        if excess <= 0:
            break
        take = min(excess, max(0, sizes[index] - int(minimums[index])))
        sizes[index] -= take
        excess -= take
    sizes[1] += max(0, int(usable) - sum(sizes))
    return sizes


def _wrap_scrollable(widget, minimum_height: int):
    """把预览卡套进一个无边框滚动区（窗口变矮时可以滚，不撑大最小高度）。"""
    scroll = QtWidgets.QScrollArea()
    scroll.setWidget(widget)
    scroll.setWidgetResizable(True)
    scroll.setFrameShape(QtWidgets.QFrame.NoFrame)
    scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
    scroll.setMinimumHeight(int(minimum_height))
    return scroll


def _normalize(values, count: int = 3, default=DEFAULT_SPLIT_RATIOS) -> tuple:
    """任意正数序列 -> 合计 :data:`RATIO_SCALE` 的 ``count`` 元整数（坏输入给兜底）。

    Args:
        values: 比例（任意正数，个数必须等于 ``count``）。
        count: 段数（3 = 预览/图/结果，2 = 表单/右栏）。
        default: 坏输入的兜底比例（必须也是 ``count`` 段，否则全平）。

    Returns:
        tuple: ``count`` 个整数，合计 :data:`RATIO_SCALE`。
    """
    fallback = [float(value) for value in default]
    if len(fallback) != int(count):              # 兜底值也得是 count 段
        fallback = [1.0] * int(count)            # pragma: no cover - 调用方写错
    try:
        numbers = [float(value) for value in values]
    except (TypeError, ValueError):
        numbers = list(fallback)
    if len(numbers) != int(count) or any(value <= 0 for value in numbers):
        numbers = list(fallback)
    total = sum(numbers)
    scaled = [int(round(value * RATIO_SCALE / total)) for value in numbers]
    scaled[-1] = RATIO_SCALE - sum(scaled[:-1])
    return tuple(int(value) for value in scaled)


def _parse_ratios(raw, count: int = 3, default=DEFAULT_SPLIT_RATIOS):
    """``QSettings`` 读出来的东西 -> ``count`` 元组；形状不对返回 ``None``。"""
    if raw is None:
        return None
    if isinstance(raw, str):
        text = raw.strip().strip("[]()")
        parts = [part for part in text.replace(";", ",").split(",")
                 if part.strip()]
    elif isinstance(raw, (list, tuple)):
        parts = list(raw)
    else:
        return None
    if len(parts) != int(count):
        return None
    try:
        values = [float(part) for part in parts]
    except (TypeError, ValueError):
        return None
    if any(value <= 0 for value in values):
        return None
    return _normalize(values, count, default)
