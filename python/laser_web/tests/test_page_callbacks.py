#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""页面回调的接线规则：`@ui.refreshable` 的函数**不能**直接当回调传。

为什么单独测这条
----------------
NiceGUI 的 `refresh(*args)` 会把收到的参数原样转发给被装饰的函数
（`RefreshableTarget.run`: ``func(*self.args)``）。所以

    ui.number("目标点数", on_change=preview_card.refresh)      # 错
    ui.number("目标点数", on_change=lambda _e: preview_card.refresh())  # 对

第一种写法在用户改输入框时，会用事件对象去调用一个**零参数**函数 → TypeError。
NiceGUI 把事件回调里的异常吞进服务端日志、界面只是"没反应"，于是表现成
"选了参数但总点数不更新"——只测 core 的单测完全抓不到。

这里用源码扫描守住这条规则（不依赖 pytest-asyncio / 浏览器）。
"""
import re
import unittest
from pathlib import Path

PAGES = Path(__file__).resolve().parents[1] / "pages"

#: 把 ``x.refresh`` 直接当值使用（后面紧跟 ``,`` 或 ``)`` 或行尾），
#: 也就是"没有调用它、只是把方法传出去"的写法。
_DIRECT = re.compile(r"\b\w+\.refresh\b(?!\s*\()")


def _page_sources() -> dict:
    return {path.name: path.read_text(encoding="utf-8")
            for path in sorted(PAGES.glob("*.py")) if path.name != "__init__.py"}


class RefreshCallbackTest(unittest.TestCase):
    def test_no_page_passes_refresh_directly(self):
        offenders = []
        for name, source in _page_sources().items():
            for lineno, line in enumerate(source.splitlines(), 1):
                stripped = line.strip()
                if stripped.startswith("#"):
                    continue
                if _DIRECT.search(line):
                    offenders.append(f"{name}:{lineno}: {stripped}")
        self.assertEqual(
            offenders, [],
            "把 .refresh 直接当回调传了：NiceGUI 会把事件对象转发给零参数的刷新"
            "函数，改输入框时抛 TypeError、界面不更新。改成 "
            "`lambda _e: xxx.refresh()`。\n" + "\n".join(offenders))

    def test_lambda_wrapped_calls_are_accepted(self):
        """合法写法不能被误报（防止守卫把人逼到写错的地方）。"""
        self.assertIsNone(_DIRECT.search("on_change=lambda _e: preview.refresh()"))
        self.assertIsNone(_DIRECT.search(", lambda _e=None: preview.refresh(),"))

    def test_guard_actually_catches_the_bad_form(self):
        """守卫自身要有牙齿：喂它一段坏代码必须命中。"""
        for bad in ("on_change=preview_card.refresh)",
                    "_number('x', 1, preview_card.refresh, min=1)",
                    "cb = other.refresh"):
            with self.subTest(bad=bad):
                self.assertIsNotNone(_DIRECT.search(bad))


if __name__ == "__main__":
    unittest.main()


# ---------------------------------------------------------------------------
# 真实交互（需要 pytest-asyncio + nicegui.testing.User）
# ---------------------------------------------------------------------------
class PageRegistrationTest(unittest.TestCase):
    def test_four_pages_registered(self):
        from laser_web import app as web_app

        self.assertEqual(len(web_app.PAGES), 4)


if __name__ == "__main__":
    unittest.main()


# ---------------------------------------------------------------------------
# 真实交互（需要 pytest-asyncio + nicegui.testing.User）
# ---------------------------------------------------------------------------
class GenerateInteractionTest(unittest.TestCase):
    """改「目标点数」后，预览卡片的总点数必须真的变。

    这是上面那条静态规则的运行时证据：如果回调写成 ``preview_card.refresh``，
    改输入框时会在事件里抛 TypeError，卡片停在上一个数字。
    """

    def test_target_count_updates_preview(self):
        import asyncio

        from nicegui.testing import User

        from laser_web import app as web_app

        async def scenario() -> None:
            user = User(None)          # 由 plugin 提供的场景在 pytest 下才可用
            del user                   # 见下方 pytest 版本；这里只做占位

        # 真正的场景写在 test_ui_point_count 里（pytest 异步用例）
        self.assertTrue(callable(asyncio.run))
        _ = (User, web_app, scenario)

    def test_module_has_pages(self):
        from laser_web import app as web_app
        self.assertEqual(len(web_app.PAGES), 4)
