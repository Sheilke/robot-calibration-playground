#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""S6e：三页的左右两栏改成**可拖的横向分割器**（``page_layout.BodySplitter``）。

现场问题：三列（左导航 / 表单 / 右侧预览+图+结果）里，左栏与右栏之间是普通
``QHBoxLayout``——只有右栏**内部**那条竖分割器（``RightSplitter``）能拖，想把
表单拖窄、把图拖宽时无处可拖。S6e 把 ``body`` 换成一个 ``QSplitter``：

* **A1**：三页都有 ``BodySplitter``（水平、两段、``childrenCollapsible=False``、
  手柄 8 px），``setSizes``/拖动真的改变两段宽度；
* **A2**：拖完把比例写进 ``QSettings``（键 ``body_splitter/<page>``），重建页面
  读回来是同一个比例；存过的比例在窄窗口装不下时回默认（判据与
  ``RightSplitter`` 那套“存过但比最小尺寸小就回默认”一致）；
* **A3**：拖到极端（左 0 / 右 0）两段都不小于各自的 ``minimumWidth()``；
* **A4**：1280×800 下拖到最左，三页的数值框仍完整（``width() >= sizeHint()``）、
  表单没有横向滚动条；
* **A5**：横向拖动不动竖分割器的比例（``protect_plot`` / ``plot_floor`` 只管
  竖直方向，横向拖完不许被程序抢回去）。

量尺寸一律用真 ``main_window.MainWindow``（与 S6c、冒烟脚本同一口径：菜单栏与
状态栏占掉的那 30 px 正是“测试里够、现场不够”的来源）。
"""
import os
import tempfile
import time
import unittest
from pathlib import Path

# 必须在建 QApplication 之前：pytest 命令行不一定带 QT_QPA_PLATFORM
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from laser_desktop import main_window, page_layout
from laser_desktop.qt_compat import Qt, QtCore, QtWidgets
from laser_desktop.tests.test_s6_common import iso_result, scan_result

_APP = None

#: 导航下标（``main_window.PAGES`` 的顺序）。
NAV_ROW = {"generate": 0, "repair": 1, "iso": 2}

#: 三页的名字（``BodySplitter`` 的 ``QSettings`` 键就是 ``body_splitter/<名字>``）。
KINDS = ("generate", "repair", "iso")

#: 1600 宽窗口下表单该拿到的宽度（px）与允许的偏差：规格里的“表单约 540 px”。
FORM_TARGET_WIDTH = 540
FORM_TARGET_DELTA = 40


def app():
    """整个测试会话共用一个 QApplication（Qt 不允许建两个）。"""
    global _APP
    if _APP is None:
        _APP = QtWidgets.QApplication.instance() or QtWidgets.QApplication(
            ["laser-desktop-s6e"])
    return _APP


def setUpModule():
    """任何 QWidget 之前先建 QApplication（否则 Qt 直接 abort）。"""
    app()


def wait(ms: int) -> None:
    """真跑 ``ms`` 毫秒事件循环（布局要至少两轮才能量尺寸）。"""
    deadline = time.monotonic() + ms / 1000.0
    while True:
        app().processEvents()
        if time.monotonic() >= deadline:
            return
        time.sleep(0.005)


def spin_boxes(page):
    """页面表单里**可见**的数值框（``键 -> 控件``）。"""
    return {key: widget for key, widget in page.fields.items()
            if isinstance(widget, QtWidgets.QAbstractSpinBox)
            and widget.isVisible()}


class _MainWindowCase(unittest.TestCase):
    """公共夹具：临时 ``QSettings`` + 一个真 ``MainWindow``。"""

    def setUp(self):
        tmp = Path(self.enterContext(tempfile.TemporaryDirectory()))
        self.settings = QtCore.QSettings(
            str(tmp / "s6e.ini"), QtCore.QSettings.IniFormat)

    # ------------------------------------------------------------ 开窗
    def open_window(self, size=(1600, 950), settings=None):
        """开一个 ``MainWindow`` 并按 ``size`` 摆好（不切页）。"""
        window = main_window.MainWindow(
            settings=self.settings if settings is None else settings)
        window.resize(*size)
        window.show()
        wait(240)
        return window

    def open_page(self, kind, size=(1600, 950), render: bool = False,
                  settings=None):
        """开窗 → 切到 ``kind`` 页 → （可选）灌一份结果；返回 (窗口, 页)。"""
        window = self.open_window(size, settings)
        window.nav.setCurrentRow(NAV_ROW[kind])
        wait(160)
        page = window.stack.currentWidget()
        if render:
            if kind == "iso":
                page.render_result(iso_result(5))
            else:
                page.render_result(scan_result(8))
            wait(300)
        return window, page

    @staticmethod
    def body(page):
        """页面里的横向分割器（``BodySplitter``）。"""
        splitter = getattr(page, "_body_splitter", None)
        assert splitter is not None, "页面没有 body 分割器"
        return splitter

    @staticmethod
    def usable(splitter) -> int:
        """两段宽度之和（``sizes()`` 的口径：已经扣掉手柄）。"""
        return splitter.width() - splitter.handleWidth()

    def drag_to(self, splitter, left_width: int) -> None:
        """模拟**用户**拖动手柄到 ``left_width``（走真的 ``splitterMoved`` 信号）。"""
        splitter.moveSplitter(int(left_width), 1)
        wait(120)


# ---------------------------------------------------------------------------
# A1：三页都有 BodySplitter，而且真的能拖
# ---------------------------------------------------------------------------
class BodySplitterDragTest(_MainWindowCase):
    """A1：``BodySplitter`` 存在、两段、水平、能拖。"""

    def test_each_page_has_a_horizontal_body_splitter(self):
        """三页的 body 都是 ``BodySplitter``：水平、两段、手柄 8 px。"""
        window = self.open_window()
        for kind in KINDS:
            window.nav.setCurrentRow(NAV_ROW[kind])
            wait(160)
            page = window.stack.currentWidget()
            splitter = self.body(page)
            self.assertIsInstance(splitter, page_layout.BodySplitter,
                                  f"{kind}：body 不是 BodySplitter")
            self.assertEqual(splitter.orientation(), Qt.Horizontal,
                             f"{kind}：分割器不是水平的")
            self.assertEqual(splitter.count(), 2, f"{kind}：不是两段")
            self.assertEqual(splitter.handleWidth(),
                             page_layout.BODY_HANDLE_WIDTH,
                             f"{kind}：手柄宽度不是 8（现场看不出能拖）")
            self.assertFalse(splitter.childrenCollapsible(),
                             f"{kind}：两段还能被拖到 0 宽")
            self.assertEqual(splitter.name, kind)
            self.assertEqual(splitter.settings_key(),
                             f"body_splitter/{kind}")
            # 左段 = 表单列（装着表单滚动区），右段 = 竖分割器
            self.assertTrue(splitter.widget(0).isAncestorOf(page.form_scroll),
                            f"{kind}：左段不是表单列")
            self.assertIs(splitter.widget(1), page._right_splitter,
                          f"{kind}：右段不是右栏")
        window.close()

    def test_set_sizes_moves_the_two_segments(self):
        """``setSizes([500, 900])`` 之后 ``sizes()`` 反映这个比例（不是布局自己说了算）。"""
        for kind in KINDS:
            window, page = self.open_page(kind)
            splitter = self.body(page)
            before = splitter.sizes()
            usable = self.usable(splitter)
            splitter.setSizes([500, 900])
            wait(200)
            sizes = splitter.sizes()
            # 下限优先于比例：repair 的表单最小宽就有 586，500 会被抬到 586
            left_min = splitter.widget(0).minimumWidth()
            expected = max(int(round(500 * usable / 1400.0)), left_min)
            # 容差 20 px：请求的 500+900 与可用宽不相等时，``QSplitter`` 会把
            # 多出的宽度按两段自己的 stretch/尺寸再分一点回来（实测 1480 可用
            # 宽、请求 500/900 → 545/935，两段各多拿 45/35）。S6e 定这条测试时
            # 三列里还没去掉那 164 px 的导航死区（可用宽只有 1316），算出来的
            # 470 直接被硬下限 524 截住，这层分摊误差才没露出来。
            self.assertAlmostEqual(
                float(sizes[0]), float(expected), delta=20.0,
                msg=f"{kind}：setSizes([500,900]) 后左段 {sizes[0]}"
                    f"（应是 ~{expected}）")
            self.assertEqual(sum(sizes), usable, f"{kind}：{sizes} 合计不对")
            # 再拖到 50/50：三页都会真的动（repair 的默认宽度本来就贴住下限了）
            splitter.setSizes([usable // 2, usable - usable // 2])
            wait(200)
            half = splitter.sizes()
            self.assertAlmostEqual(float(half[0]), float(usable // 2), delta=2.0,
                                   msg=f"{kind}：拖到中间后 {half}")
            self.assertNotEqual(half, before, f"{kind}：setSizes 没改变宽度")
            window.close()

    def test_default_split_keeps_the_form_at_its_natural_width(self):
        """首开左栏 = max(41% 比例, 表单自然宽度)：1600 下与改造前逐像素一致。"""
        window = self.open_window()
        for kind in KINDS:
            window.nav.setCurrentRow(NAV_ROW[kind])
            wait(160)
            page = window.stack.currentWidget()
            splitter = self.body(page)
            sizes = splitter.sizes()
            natural = splitter.natural_left_width()
            left_min, right_min = splitter.segment_minimums()
            usable = self.usable(splitter)
            wanted = max(
                int(round(usable * page_layout.DEFAULT_BODY_RATIOS[0]
                          / page_layout.RATIO_SCALE)),
                natural)
            expected = max(min(wanted, usable - right_min), left_min)
            self.assertAlmostEqual(
                float(sizes[0]), float(expected), delta=2.0,
                msg=f"{kind}：默认左栏 {sizes[0]}（应是 {expected}：自然宽 "
                    f"{natural}，下限 {left_min}）")
            self.assertGreaterEqual(sizes[0], left_min)
            self.assertGreaterEqual(
                sizes[0], FORM_TARGET_WIDTH - FORM_TARGET_DELTA,
                f"{kind}：默认左栏 {sizes[0]} px 比规格的 ~{FORM_TARGET_WIDTH} px "
                f"窄太多")
            self.assertGreaterEqual(natural, page.form_scroll.minimumWidth(),
                                    f"{kind}：表单的自然宽度小于它的硬下限")
            # “1600 宽下左栏约 34%”：落在 30%–45% 之间（表单自然宽度决定上限）
            share = sizes[0] / float(window.width())
            self.assertGreater(share, 0.30, f"{kind}：左栏只占 {share:.0%}")
            self.assertLess(share, 0.45, f"{kind}：左栏占了 {share:.0%}")
        window.close()

    def test_each_page_can_be_dragged_narrower(self):
        """三页都真的能拖窄（默认左栏高于自己的最小值，不是一开局就贴住下限）。"""
        for kind in KINDS:
            window, page = self.open_page(kind)
            splitter = self.body(page)
            start = splitter.sizes()[0]
            left_min = splitter.widget(0).minimumWidth()
            self.assertGreater(
                start, left_min + 20,
                f"{kind}：默认左栏 {start} 与最小值 {left_min} 差不多，拖不动")
            self.drag_to(splitter, 1)
            sizes = splitter.sizes()
            self.assertLessEqual(sizes[0], left_min + 2,
                                 f"{kind}：拖到最左后 {sizes}")
            self.assertGreaterEqual(sizes[1], splitter.widget(1).minimumWidth())
            window.close()

    def test_user_drag_is_remembered_in_settings(self):
        """真的拖一下（``moveSplitter``）：``QSettings`` 里出现这一页的比例。"""
        for kind in KINDS:
            window, page = self.open_page(kind)
            splitter = self.body(page)
            self.assertFalse(splitter.user_moved, f"{kind}：开局就算“拖过”")
            self.assertIsNone(splitter.saved_ratios(),
                              f"{kind}：还没拖就有存过的比例")
            self.drag_to(splitter, 620)
            self.assertTrue(splitter.user_moved,
                            f"{kind}：拖了却没记住“用户拖过”")
            stored = splitter.saved_ratios()
            self.assertIsNotNone(stored, f"{kind}：拖完没写进 QSettings")
            self.assertEqual(tuple(stored), tuple(splitter.live_ratios()),
                             f"{kind}：存下的比例与屏幕上的不一致 {stored} vs "
                             f"{splitter.live_ratios()}")
            self.assertEqual(len(stored), 2)
            self.assertEqual(sum(stored), page_layout.RATIO_SCALE)
            window.close()


# ---------------------------------------------------------------------------
# A2：QSettings 往返
# ---------------------------------------------------------------------------
class BodySplitterSettingsTest(_MainWindowCase):
    """A2：拖完 → 重建页面 → 读回同一个比例（宽窗口）／回默认（窄窗口）。"""

    def test_rebuilt_page_restores_the_dragged_ratio(self):
        window, page = self.open_page("generate")
        splitter = self.body(page)
        self.drag_to(splitter, 600)
        dragged = splitter.sizes()
        ratios = tuple(splitter.ratios)
        window.close()

        rebuilt_window, rebuilt = self.open_page("generate")
        restored = self.body(rebuilt)
        self.assertTrue(restored._from_settings,
                        "重建后没读回存过的比例")
        self.assertEqual(tuple(restored.ratios), ratios,
                         f"比例没回来：{restored.ratios} vs {ratios}")
        sizes = restored.sizes()
        self.assertAlmostEqual(float(sizes[0]), float(dragged[0]), delta=2.0,
                               msg=f"左段宽度没回来：{sizes} vs {dragged}")
        self.assertAlmostEqual(float(sizes[1]), float(dragged[1]), delta=2.0,
                               msg=f"右段宽度没回来：{sizes} vs {dragged}")
        rebuilt_window.close()

    def test_stored_ratio_too_narrow_for_the_window_falls_back(self):
        """存过的比例在 1280 下装不下表单 → 回默认比例（不硬挤、不裁控件）。"""
        self.settings.setValue("body_splitter/generate", [800, 200])
        self.settings.sync()
        window, page = self.open_page("generate", page_layout.MINIMUM_WINDOW_SIZE)
        splitter = self.body(page)
        self.assertTrue(splitter._stored_too_narrow,
                        "存过的 800/200 在 1280 下明明装不下，却没标记回默认")
        left_min, _right_min = splitter.segment_minimums()
        self.assertGreaterEqual(splitter.sizes()[0], left_min,
                                f"左段被压到 {splitter.sizes()[0]} < {left_min}")
        self.assertGreaterEqual(
            page.form_scroll.width(),
            page.form_scroll.minimumSizeHint().width() - 1,
            "表单被压到看不全")
        window.close()

    def test_broken_settings_value_is_ignored(self):
        """坏配置（段数不对 / 非数字）不许把布局带沟里。"""
        for bad in ("[800]", "abc,def", [1, 2, 3], ""):
            self.settings.setValue("body_splitter/repair", bad)
            self.settings.sync()
            window, page = self.open_page("repair")
            splitter = self.body(page)
            self.assertFalse(splitter._from_settings,
                             f"坏配置 {bad!r} 被当成有效比例")
            self.assertEqual(tuple(splitter.ratios),
                             page_layout._normalize(
                                 page_layout.DEFAULT_BODY_RATIOS, 2,
                                 page_layout.DEFAULT_BODY_RATIOS),
                             f"坏配置 {bad!r} 之后没回默认比例")
            window.close()


# ---------------------------------------------------------------------------
# A3：拖到极端也不裁内容
# ---------------------------------------------------------------------------
class BodySplitterMinimumTest(_MainWindowCase):
    """A3：拖到最左/最右，两段都不小于各自的 ``minimumWidth()``。"""

    def test_extreme_drags_stop_at_the_minimum_widths(self):
        for kind in KINDS:
            window, page = self.open_page(kind)
            splitter = self.body(page)
            left_min, right_min = splitter.segment_minimums()
            self.assertEqual(splitter.widget(0).minimumWidth(), left_min,
                             f"{kind}：左段的 minimumWidth 没钉到下限")
            self.assertEqual(splitter.widget(1).minimumWidth(), right_min,
                             f"{kind}：右段的 minimumWidth 没钉到下限")
            usable = self.usable(splitter)

            splitter.setSizes([0, usable])
            wait(160)
            sizes = splitter.sizes()
            self.assertGreaterEqual(
                sizes[0], splitter.widget(0).minimumWidth(),
                f"{kind}：拖到最左后左段 {sizes[0]} < minimumWidth "
                f"{splitter.widget(0).minimumWidth()}")
            self.assertLessEqual(sizes[0], left_min + 2,
                                 f"{kind}：拖到最左却没贴住下限 {sizes}")

            splitter.setSizes([usable, 0])
            wait(160)
            sizes = splitter.sizes()
            self.assertGreaterEqual(
                sizes[1], splitter.widget(1).minimumWidth(),
                f"{kind}：拖到最右后右段 {sizes[1]} < minimumWidth "
                f"{splitter.widget(1).minimumWidth()}")
            self.assertLessEqual(sizes[1], right_min + 2,
                                 f"{kind}：拖到最右却没贴住下限 {sizes}")
            window.close()

    def test_minimum_widths_come_from_the_content(self):
        """左段 = 表单的最小宽度，右段 = 右栏（图/结果）的最小宽度。"""
        for kind in KINDS:
            window, page = self.open_page(kind)
            splitter = self.body(page)
            left_min, right_min = splitter.segment_minimums()
            self.assertEqual(left_min,
                             max(page_layout.BODY_LEFT_FLOOR_WIDTH,
                                 page._body_splitter.widget(0)
                                 .minimumSizeHint().width()),
                             f"{kind}：左段下限不是表单的最小宽度")
            self.assertEqual(right_min,
                             max(page_layout.BODY_RIGHT_FLOOR_WIDTH,
                                 page._right_splitter.minimumSizeHint().width()),
                             f"{kind}：右段下限不是右栏的最小宽度")
            self.assertLessEqual(
                splitter.children_min_width(),
                page_layout.MINIMUM_WINDOW_SIZE[0],
                f"{kind}：两段下限之和装不下 1280 宽的窗口")
            window.close()

    def test_window_minimum_width_still_fits_1280(self):
        """两段下限之和不能把窗口撑到 1280 以上（撑大就说明哪一段下限写大了）。"""
        window = self.open_window(page_layout.MINIMUM_WINDOW_SIZE)
        hint = window.minimumSizeHint()
        self.assertLessEqual(hint.width(), page_layout.MINIMUM_WINDOW_SIZE[0],
                             f"窗口最小宽度 {hint.width()} 超过 1280")
        window.close()


# ---------------------------------------------------------------------------
# A4：1280×800 拖到最左，数值框与表单都不许裁
# ---------------------------------------------------------------------------
class MinimumWindowNoClipTest(_MainWindowCase):
    """A4：1280×800 + 左栏拖到最窄：数值框 ``width() >= sizeHint()``、无横滚条。"""

    def _check(self, kind, label):
        window, page = self.open_page(kind, page_layout.MINIMUM_WINDOW_SIZE)
        splitter = self.body(page)
        if kind != "iso":
            page.fields["domain_axes"].setChecked(True)   # 「六个轴边界」面板
            wait(120)
        splitter.setSizes([0, 10 ** 6])                   # 拖到最左
        wait(200)

        form = page.form_scroll
        self.assertEqual(
            form.horizontalScrollBar().maximum(), 0,
            f"{label}：表单出现横向滚动条（maximum="
            f"{form.horizontalScrollBar().maximum()}，视口 "
            f"{form.viewport().width()} < 内容 "
            f"{form.widget().minimumSizeHint().width()}）")
        boxes = spin_boxes(page)
        self.assertGreater(len(boxes), 3, f"{label}：没找到数值框")
        clipped = {key: (box.width(), box.sizeHint().width())
                   for key, box in boxes.items()
                   if box.width() < box.sizeHint().width()}
        self.assertEqual(clipped, {},
                         f"{label}：数值框被压窄（键: 实际/需要）={clipped}")
        # 拖到最左之后左段就是表单的最小宽度
        self.assertGreaterEqual(
            splitter.sizes()[0], splitter.widget(0).minimumWidth(),
            f"{label}：左段 {splitter.sizes()} 小于 minimumWidth")
        window.close()

    def test_generate_page(self):
        self._check("generate", "生成页 1280×800")

    def test_repair_page(self):
        self._check("repair", "修复页 1280×800")

    def test_iso_page(self):
        self._check("iso", "ISO 页 1280×800")

    def test_dragged_right_keeps_the_result_area_usable(self):
        """拖到最右（右栏最窄）：右段仍不小于右栏的最小宽度。"""
        window, page = self.open_page("repair",
                                      page_layout.MINIMUM_WINDOW_SIZE)
        splitter = self.body(page)
        splitter.setSizes([10 ** 6, 0])
        wait(200)
        sizes = splitter.sizes()
        self.assertGreaterEqual(sizes[1], splitter.widget(1).minimumWidth(),
                                f"右栏被拖到 {sizes[1]} < "
                                f"{splitter.widget(1).minimumWidth()}")
        self.assertGreater(page._right_splitter.width(), 0)
        window.close()


# ---------------------------------------------------------------------------
# A5：横向拖动不动竖分割器
# ---------------------------------------------------------------------------
class VerticalSplitterUnaffectedTest(_MainWindowCase):
    """A5：横向拖动之后竖分割器的比例不变（``protect_plot`` 只管竖直）。"""

    def _check(self, kind, label):
        window, page = self.open_page(kind, page_layout.DEFAULT_WINDOW_SIZE,
                                      render=True)
        vertical = page._right_splitter
        before = list(vertical.sizes())
        ratios_before = tuple(vertical.live_ratios())
        body = self.body(page)
        self.drag_to(body, body.widget(0).minimumWidth())
        wait(240)
        after = list(vertical.sizes())
        ratios_after = tuple(vertical.live_ratios())
        for index, (old, new) in enumerate(zip(before, after)):
            self.assertAlmostEqual(
                float(new), float(old), delta=4.0,
                msg=f"{label}：横拖之后竖分割第 {index} 段 {new}（原 {old}，"
                    f"分割 {before} -> {after}）")
        self.assertEqual(ratios_after, ratios_before,
                         f"{label}：竖分割比例变了 {ratios_before} -> "
                         f"{ratios_after}")
        self.assertFalse(vertical.user_moved,
                         f"{label}：横拖把“用户拖过竖分割条”给置上了")
        self.assertFalse(vertical.protect_plot(),
                         f"{label}：横拖之后 protect_plot 还在抢高度")
        window.close()

    def test_generate_page(self):
        self._check("generate", "生成页")

    def test_repair_page(self):
        self._check("repair", "修复页")

    def test_iso_page(self):
        self._check("iso", "ISO 页")

    def test_horizontal_drag_survives_a_window_resize(self):
        """窗口 resize 时横向比例按比例保持（不许被程序重算回默认）。"""
        window, page = self.open_page("generate")
        body = self.body(page)
        self.drag_to(body, 600)
        left_before = body.sizes()[0]
        usable_before = self.usable(body)
        dragged = tuple(body.live_ratios())
        window.resize(1480, 930)
        wait(240)
        self.assertEqual(tuple(body.live_ratios()), dragged,
                         f"缩小窗口后横向比例变了 {dragged} -> "
                         f"{body.live_ratios()}")
        expected = left_before * self.usable(body) / float(usable_before)
        self.assertAlmostEqual(
            float(body.sizes()[0]), expected, delta=12.0,
            msg=f"缩小窗口后左段 {body.sizes()[0]}（按比例应 ~{expected:.0f}）")
        window.resize(1600, 950)
        wait(240)
        self.assertEqual(tuple(body.live_ratios()), dragged,
                         "放大回默认尺寸后横向比例变了")
        self.assertAlmostEqual(float(body.sizes()[0]), float(left_before),
                               delta=4.0,
                               msg=f"放大回默认尺寸后左段 {body.sizes()[0]}"
                                   f"（原 {left_before}）")
        window.close()


if __name__ == "__main__":
    unittest.main()
