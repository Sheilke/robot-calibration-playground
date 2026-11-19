# -*- mode: python ; coding: utf-8 -*-
"""PyInstaller 规格：桌面版 ``--onedir`` 打包（Windows 10/11 x64）。

    pyinstaller packaging/desktop.spec --noconfirm

产物：``dist/LaserCalibration/LaserCalibration.exe`` + ``dist/LaserCalibration/_internal/``。
本文件**只能在 Windows 目标机上跑**：PyInstaller 不是交叉编译器，它复制的是
"当前正在运行的那个解释器"连同 ``.pyd`` / ``.dll`` 与平台启动器，Linux 上执行
只会得到一堆 ELF。

为什么是 onedir 而不是 onefile（写死在这里，别再改回去）
--------------------------------------------------------
``--onefile`` 每次启动都要把 400+ MB 解到 ``%TEMP%\\_MEIxxxx``：冷启动多 2–5 s，
而且对 QtWebEngine 是**功能性**问题——``QtWebEngineProcess.exe`` 要在启动那一刻
按 ``qt.conf`` 的相对路径找到 ``resources/`` 与 ``Qt6*.dll``，解包目录一变就白屏。

这个 spec 干四件事
------------------
1. **只读资产**：机器人模型（``data/configs`` 顶层的 ``*.urdf`` / ``*_dh.json``）、
   离线 ``plotly.min.js`` 与**授权公钥**（``data/assets/license_public.pem``，
   有就收）→ ``_internal/assets/``，正好是
   :func:`laser_desktop.paths.asset_root` 在冻结状态下要找的位置。
2. **QtWebEngine 运行时**：交给 PyInstaller 自带的
   ``hook-PySide6.QtWebEngineCore.py``（它调
   ``pyside6_library_info.collect_qtwebengine_files()``），再补一次
   ``collect_data_files`` 兜底，最后**逐项核对**关键文件在不在 TOC 里。
   全程没有手写任何绝对路径。
3. **瘦身**：丢掉 QtWebEngine 的 debug 资源（84.2 MB）与 50 个用不到的
   locale（44 MB），只留简体/繁体中文与英文。
4. **自检**：关键文件缺失时**让构建当场失败**并打印人话，而不是打出一个白屏的包。

两个容易踩的坑（代码里都处理了）
--------------------------------
* ``a.datas`` / ``a.binaries`` 的 TOC 条目是 ``(目标名, 源路径, 类型)``——**目标名在
  第 0 位**，不是 ``(src, dest)``。过滤时按第 0 位判，否则会把体积统计算反。
* 入口不能直接用 ``python/laser_desktop/__main__.py``：它内部是
  ``from . import paths`` 这样的**相对导入**，而 PyInstaller 把入口脚本当作
  顶层 ``__main__`` 执行（``__package__`` 为空），相对导入会直接
  ``ImportError: attempted relative import with no known parent package``。
  所以这里在 ``workpath`` 里生成一个 3 行的绝对导入入口（见 §5），
  不去改 ``laser_desktop`` 的源码。
"""
import os

# ---------------------------------------------------------------------------
# 0. 锚点
# ---------------------------------------------------------------------------
# spec 由 PyInstaller 用 exec() 执行，命名空间里**没有 __file__**，只有
# SPECPATH（spec 所在目录）与 workpath（构建临时目录，默认 build/<spec 名>）。
_SPEC_DIR = globals().get("SPECPATH") or os.getcwd()
REPO_ROOT = os.path.dirname(os.path.abspath(_SPEC_DIR))
PY_SRC = os.path.join(REPO_ROOT, "python")
WORK_DIR = globals().get("workpath") or os.path.join(REPO_ROOT, "build", "desktop")
os.makedirs(WORK_DIR, exist_ok=True)

APP_NAME = "LaserCalibration"
APP_VERSION = "0.9.0"
APP_PUBLISHER = "3S Robotics / robot-calibration-playground"

CONFIG_DIR = os.path.join(REPO_ROOT, "data", "configs")
ASSET_PLOTLY_DIR = os.path.join(REPO_ROOT, "data", "assets", "plotly")


def _log(message):
    """构建日志（pyinstaller 会把它和 INFO 一起写进 build/desktop/*.log）。"""
    print("[desktop.spec] " + message)


def _missing(what, path):
    raise SystemExit(
        "[desktop.spec] 找不到%s：%s\n"
        "  先跑 packaging/build_windows.ps1（它会建 venv 并补齐资产），"
        "或在仓库根执行：\n"
        "    pyinstaller packaging/desktop.spec --noconfirm" % (what, path))


# ---------------------------------------------------------------------------
# 1. 只读资产：机器人模型 + 离线 plotly.min.js → _internal/assets/
# ---------------------------------------------------------------------------
# 目标目录必须是 "assets"：laser_desktop/paths.py 里 asset_root() 在冻结状态下
# 返回 <_MEIPASS>/assets，config_dir() 再拼 data/configs。页面调
# core.list_robots(paths.asset_root())，所以模型落在 assets/data/configs 才对得上。
datas = []
_model_count = 0
if os.path.isdir(CONFIG_DIR):
    for name in sorted(os.listdir(CONFIG_DIR)):
        if not (name.endswith(".urdf") or name.endswith("_dh.json")):
            continue                       # robot_sim/*.json、*_scan.json 故意不收
        datas.append((os.path.join(CONFIG_DIR, name), "assets/data/configs"))
        _model_count += 1
if _model_count == 0:
    _missing("机器人模型（data/configs/*.urdf、*_dh.json）", CONFIG_DIR)

# 收什么、为什么不收相邻的：
#   * *.urdf            —— core.list_robots() 认的模型格式，Nexbot6R 能直接读；
#   * *_dh.json         —— DH 覆盖文件（hyundai_general6s_dh.json），随 urdf 一起用；
#   * robot_sim/*.json  —— 仿真器资产（3 个控制器 JSON），**不是**机器人模型，
#                           list_robots() 试加载必然 KeyError，收进去只会让下拉框
#                           多三个"不可用"的条目；
#   * *_scan.json       —— 某次扫描的记录，运行期产物，不属于只读资产。
_log("机器人模型 %d 个 -> assets/data/configs" % _model_count)

# plotly.min.js（4.6 MB）：**离线画图的前提**。运行时 plot_view 用
# plotly.offline.get_plotlyjs() 取 JS 写进 cache_root()/plot/，取的是
# plotly 包自带的 package_data/plotly.min.js（见 §3）；这里的副本是**第二个**
# 来源，给"plotly 包自带的被人删了/以后换成自家 JS"留条活路，
# 也让安装包里有一份看得见、能单独替换的 JS。
_plotly_asset = os.path.join(ASSET_PLOTLY_DIR, "plotly.min.js")
_plotly_source = None
if os.path.isfile(_plotly_asset):
    _plotly_source = _plotly_asset
else:
    # 仓库里没有（data/assets/ 是构建产物，不进库）：从已安装的 plotly 里取。
    try:
        import plotly
        _candidate = os.path.join(os.path.dirname(plotly.__file__),
                                  "package_data", "plotly.min.js")
        if os.path.isfile(_candidate):
            _plotly_source = _candidate
            os.makedirs(ASSET_PLOTLY_DIR, exist_ok=True)
            with open(_candidate, "rb") as src, open(_plotly_asset, "wb") as dst:
                dst.write(src.read())
            _log("data/assets/plotly/plotly.min.js 不存在，已从 plotly %s 复制"
                 % getattr(plotly, "__version__", "?"))
    except ImportError:
        pass
if _plotly_source is None:
    _missing("plotly.min.js（data/assets/plotly/ 或已安装的 plotly 包）",
             os.path.join(ASSET_PLOTLY_DIR, "plotly.min.js"))
datas.append((_plotly_asset, "assets/plotly"))
_js_size = os.path.getsize(_plotly_asset)
if _js_size < 3.5 * 1024 * 1024:
    # 4.6 MB 的东西只剩几十 KB，多半是空文件/被 LFS 占位符替换了
    raise SystemExit("[desktop.spec] plotly.min.js 只有 %.1f KB，明显不完整：%s"
                     % (_js_size / 1024.0, _plotly_asset))
_log("plotly.min.js %.2f MB -> assets/plotly" % (_js_size / 1024.0 / 1024.0))

# ---------------------------------------------------------------------------
# 1b. 离线授权的**公钥**（私钥绝不进包）
# ---------------------------------------------------------------------------
# 启动闸门（laser_desktop/licensing.py）在 asset_root() 下找 license_public.pem，
# 冻结后就是 _internal/assets/license_public.pem，所以目标目录必须写 "assets"。
#
# 为什么没有这份文件**不算构建失败**：换一把私钥签发、先出一个只给内测机的包，
# 都是正常需求；而把"缺公钥"做成硬失败，会让人为了赶进度去注释掉这行检查。
# 但包里没公钥 = 装到客户机上启动就报「程序缺少内置公钥」，所以这里喊一嗓，
# 再由 verify_dist.py 的 [WARN] 与 build_windows.ps1 的醒目提示各提醒一遍。
#
# 私钥（*.pem 里带 PRIVATE KEY 的那个）**绝不能**出现在这里：它一泄露，谁都能
# 自己签授权。仓库里也不应该有——签发在离线机器上做。
_PUBLIC_KEY_CANDIDATES = (
    os.path.join(REPO_ROOT, "license_public.pem"),
    os.path.join(REPO_ROOT, "data", "assets", "license_public.pem"),
)
_PUBLIC_KEY_SRC = next((path for path in _PUBLIC_KEY_CANDIDATES
                        if os.path.isfile(path)), None)
# **公钥编进代码**，不再只放成可替换的文件。
#
# 实测过的绕过：随包的 license_public.pem 是一个普通文件，攻击者把它换成自己的
# 公钥、再用自己的私钥签一张 2099 年的证，校验就通过了——一行代码都不用改。
# 所以构建时把公钥写进 laser_desktop/_embedded_key.py，它会进 PYZ；冻结运行时
# licensing.public_key() 只认这一份，不读 assets 下的文件。
_EMBEDDED_KEY_MODULE = os.path.join(REPO_ROOT, "python", "laser_desktop",
                                    "_embedded_key.py")
if _PUBLIC_KEY_SRC:
    with open(_PUBLIC_KEY_SRC, "rb") as _fp:
        _pem = _fp.read()
    with open(_EMBEDDED_KEY_MODULE, "w", encoding="utf-8") as _fp:
        _fp.write("# 由 packaging/desktop.spec 生成，不要手改、不要提交。\n"
                  "# 冻结运行时 licensing.public_key() 只读这一份。\n"
                  "PUBLIC_KEY_PEM = %r\n" % _pem)
    _log("授权公钥 %d B -> laser_desktop/_embedded_key.py（编进代码，%s）"
         % (len(_pem), os.path.basename(_PUBLIC_KEY_SRC)))
else:
    _log("!! 没有 license_public.pem（仓库根或 data/assets/）：这个包将不含公钥，"
         "客户机上启动会直接报「程序缺少内置公钥」（只适合内测包）")

# ---------------------------------------------------------------------------
# 2. matplotlib：mpl-data（字体、样式表、teardown.cfg）
# ---------------------------------------------------------------------------
# PyInstaller 的 hook-matplotlib.py 已经会收 mpl-data，这里再显式收一次是同名
# 去重（TOC 按目标名去重），成本为零、省得将来换 PyInstaller 版本后
# "FileNotFoundError: .../mpl-data/fonts/ttf/DejaVuSans.ttf"（计划 §5.7 的 T4）。
#
# **中文字体不捆**，理由是可验证的：
#   * Qt 侧（控件、对话框）走 qt_compat.apply_chinese_font()，Windows 上用系统自带
#     的 Microsoft YaHei UI，字体文件在 C:\Windows\Fonts，从来不需要随包分发；
#   * matplotlib 侧（core 写 plot.png）**全仓库没有设置过 rcParams["font.sans-serif"]**，
#     而 plot.png 里的文字实测只有 "X (mm)" / "waypoints 27/27" 这类 ASCII，
#     DejaVuSans（mpl-data 自带）足够；打不了中文是现状，不是打包引入的。
#   真要捆一份 Noto Sans CJK SC 是 +20 MB，且 Python 侧没有任何 addfont() 调用
#   会用到它——所以不做。（若将来 PNG 上出现中文标题，正确做法是在 core 里
#   点名字体族，而不是往包里塞字体文件。）
from PyInstaller.utils.hooks import collect_data_files

datas += collect_data_files("matplotlib")
_log("matplotlib mpl-data 已收集")

# ---------------------------------------------------------------------------
# 3. plotly 的 JS 数据（运行时 get_plotlyjs() 真读这个文件）
# ---------------------------------------------------------------------------
# plotly.offline.get_plotlyjs() 的实现是
#     pkgutil.get_data("plotly", "package_data/plotly.min.js")
# 冻结后 pkgutil 走 PyInstaller 的 FrozenImporter.get_data，读的就是
# _internal/plotly/package_data/plotly.min.js。少了它，壳页里的
# <script src="plotly.min.js"> 是空的 → 白屏（P2 那类"图上什么都没有"）。
# widgetbundle.js（5 MB，Jupyter 部件用）与 Jupyter 的 labextension 不收。
_plotly_js = [(src, dst) for src, dst in
              collect_data_files("plotly", includes=["package_data/*.js"])
              if not os.path.basename(src).startswith("widgetbundle")]
datas += _plotly_js
_log("plotly 数据 %d 个（已剔除 widgetbundle.js）" % len(_plotly_js))

# ---------------------------------------------------------------------------
# 4. QtWebEngine 运行时：hook 为主，collect 兜底，最后逐项核对
# ---------------------------------------------------------------------------
# 谁把 QtWebEngineProcess.exe / resources/ / translations/qtwebengine_locales/
# 收进来的：PyInstaller 自带 hook（6.16.0 实测源码）
#   PyInstaller/hooks/hook-PySide6.QtWebEngineCore.py
#     -> pyside6_library_info.collect_qtwebengine_files()
#        PyInstaller/utils/hooks/qt/__init__.py:1050-1200
# 它在 Windows 分支上收三样（目标都在 _internal/PySide6/ 下）：
#   TranslationsPath/qtwebengine_locales -> translations/qtwebengine_locales
#   DataPath/resources                  -> resources          （含 icudtl.dat）
#   LibraryExecutablesPath/QtWebEngineProcess* -> .            （孑进程）
#   + 自动生成 qt.conf（Qt ≥ 6.3 的 wheel 里没有这个文件了）
# 这个 hook 只在"程序真的 import 了 PySide6.QtWebEngineWidgets"时才跑，
# 所以下面 hiddenimports 里点名它（计划 §4.6 的 W2）。
#
# 那为什么还要 collect_data_files？因为 hook 的输出取决于 PyInstaller 版本，
# 而漏收的症状是"白屏 + 没有报错"，最难查。这两行是**幂等**的（TOC 按目标名
# 去重）：hook 收了就合并，hook 哪天漏了就由这里补上。注意只 include 了
# resources 与 qtwebengine_locales 两个目录——不写 collect_data_files("PySide6")
# 那种整包收：那会把 84.2 MB 的 *.debug.pak 和 53 个 locale 全拉进来
# （虽然 §7 会把它们剪掉，但没必要先收再加）。
datas += collect_data_files("PySide6",
                            includes=["resources/*",
                                      "translations/qtwebengine_locales/*"])
_log("PySide6 resources / qtwebengine_locales 已收集（与 hook 输出去重）")

# ---------------------------------------------------------------------------
# 5. 入口脚本（绝对导入；见文件头"两个容易踩的坑"）
# ---------------------------------------------------------------------------
# 生成物落在 workpath（默认 build/desktop/），不进仓库、不会被当成源码。
_ENTRY_PATH = os.path.join(WORK_DIR, "entry_laser_desktop.py")
with open(_ENTRY_PATH, "w", encoding="utf-8") as fp:
    fp.write(
        "# -*- coding: utf-8 -*-\n"
        "# 由 packaging/desktop.spec 生成，不要手改（每次 pyinstaller 都会重写）。\n"
        "import multiprocessing\n"
        "import os\n"
        "import sys\n"
        "\n"
        "\n"
        "def _check_frozen_repo_root():\n"
        "    \"\"\"构建期校验：``core._REPO_ROOT`` 必须已经等于资产根。\n"
        "\n"
        "    历史：core.py 曾用 ``Path(__file__).resolve().parents[2]`` 反推仓库根，\n"
        "    冻结后 ``__file__`` 是 ``<_MEIPASS>/laser_web/core``（无 ``.py``），\n"
        "    数出来是**安装目录**，于是每个模型的相对路径都拼错、一点解算就\n"
        "    FileNotFoundError。现在正解在源码里：``laser_web/paths.py``（见\n"
        "    ``core._REPO_ROOT = _asset_root()``）。\n"
        "\n"
        "    这里**不再替它赋值**，只做断言：两边不一致就当场把话说清楚。\n"
        "    替它赋值会静默掩盖两个模块算出不同答案这件事——那种沉默比报错贵。\n"
        "    \"\"\"\n"
        "    from laser_desktop import paths\n"
        "    import laser_web.core as core\n"
        "    if core._REPO_ROOT != paths.asset_root():\n"
        "        raise SystemExit(\n"
        "            '打包中止：laser_web.core._REPO_ROOT=%s 与 asset_root()=%s 不一致，'\n"
        "            '冻结后所有模型路径都会拼错。修 laser_web/paths.py，'\n"
        "            '不要在 spec 里下手。'\n"
        "            % (core._REPO_ROOT, paths.asset_root()))\n"
        "\n"
        "\n"
        "def _prepare():\n"
        "    # 冻结后没人给我们设 MPLBACKEND：先钉死 Agg，免得 matplotlib 自己\n"
        "    # 去猜一个 GUI 后端（包里没有 TkAgg，猜错就是 ImportError）。\n"
        "    os.environ.setdefault('MPLBACKEND', 'Agg')\n"
        "    # WebEngine 的环境变量必须在建 QApplication 之前设好（配置函数可重复调）。\n"
        "    from laser_desktop.qt_compat import configure_webengine_environment\n"
        "    configure_webengine_environment()\n"
        "    _check_frozen_repo_root()\n"
        "\n"
        "\n"
        "def _main():\n"
        "    # --windowed 下没有控制台：sys.stdout/stderr 是 None，任何 print 都会\n"
        "    # AttributeError（计划 §5.7 T3）。先兜住，再进真正的入口。\n"
        "    for name in ('stdout', 'stderr'):\n"
        "        if getattr(sys, name) is None:\n"
        "            setattr(sys, name, open(os.devnull, 'w'))\n"
        "    from laser_desktop.__main__ import main\n"
        "    return main()\n"
        "\n"
        "\n"
        "if __name__ == '__main__':\n"
        "    # 现在没用进程池，但冻结后 fork/spawn 语义不同，这行是零成本的保险\n"
        "    # （计划 §5.7 T13）。\n"
        "    multiprocessing.freeze_support()\n"
        "    _prepare()\n"
        "    sys.exit(_main())\n")
_log("入口脚本：%s" % _ENTRY_PATH)

# ---------------------------------------------------------------------------
# 6. 分析与排除
# ---------------------------------------------------------------------------
hiddenimports = [
    # 交付用的 Qt 绑定是 PySide6（LGPL）。core 要靠这个 import 触发 QtWebEngine
    # 的 hook（W2）；import 在 qt_compat 的模块顶层，这里再点名一次是保险。
    "PySide6.QtWebEngineWidgets",
    "PySide6.QtWebEngineCore",
    "PySide6.QtWebChannel",          # qwebchannel.js 在它的 Qt 资源里
    "PySide6.QtPrintSupport",        # QtWebEngineWidgets 的硬依赖（hook 也会加）
    # 函数内部 import 的三个（PyInstaller 的静态分析本来能看见，但它们是
    # "只有点到那个按钮才崩"的类型，多写两行比事后查便宜 —— 计划 §5.7 T12）。
    "robot_model.json_to_urdf",
    "excitation_trajectory.export_gp",
    "excitation_trajectory.aabb_snake_scan",
    "laser_web.core",
    "laser_web.jobs",
    "laser_web.figures",
    "licensing.hwid",
    # 启动闸门里的 import 都在函数体内（延迟到真要校验时才花这个钱）：
    # 静态分析能看见，但它们是"不点就不知道"的那一类，点名更保险。
    # cryptography 带 Rust 后端（_rust.pyd），靠 PyInstaller 自带的
    # hook-cryptography 收，不要在这里手写二进制。
    "licensing.license",
    "cryptography",
    # matplotlib 的 Agg：源码里是 matplotlib.use("Agg")（字符串），
    # hooksconfig 已经钉死后端，这里再点名一次。
    "matplotlib.backends.backend_agg",
]

excludes = [
    # 网页版才用：桌面版不 import nicegui（否则 +27 MB）
    "nicegui",
    # 标准库里体积大且完全用不到的部分
    "tkinter", "_tkinter", "turtle", "turtledemo",
    "test", "unittest", "doctest", "pydoc", "idlelib", "lib2to3",
    # 只在开发/测试时用的
    "pytest", "_pytest", "IPython", "Sphinx",
    # 反向确认：**不要**两套 Qt 绑定同时进包（PyInstaller 的
    # ensure_single_qt_bindings_package 也会拦，这里写死免得报警告）
    "PyQt5", "PyQt6", "PySide2",
    # Qt 模块：这些连 WebEngine 自己都不依赖（见下面"为什么不排除 QtQuick"）
    "PySide6.Qt3DAnimation", "PySide6.Qt3DCore", "PySide6.Qt3DExtras",
    "PySide6.Qt3DInput", "PySide6.Qt3DLogic", "PySide6.Qt3DRender",
    "PySide6.QtMultimedia", "PySide6.QtMultimediaWidgets",
    "PySide6.QtCharts", "PySide6.QtDataVisualization",
    "PySide6.QtGraphs", "PySide6.QtGraphsWidgets",
    "PySide6.QtDesigner", "PySide6.QtUiTools",
    "PySide6.QtWebEngineQuick",      # 只用 Widgets 版
    "PySide6.QtBluetooth", "PySide6.QtNfc", "PySide6.QtSensors",
    "PySide6.QtSerialPort", "PySide6.QtSerialBus",
    "PySide6.QtRemoteObjects", "PySide6.QtScxml",
    "PySide6.QtPdf", "PySide6.QtPdfWidgets",
]

# 为什么不把 QtQuick / QtQml 放进 excludes（计划里说"除非 WebEngine 依赖它"）：
# **核实过，它依赖。** 直接扫 Windows 版 wheel 里两个 DLL 的导入表：
#   Qt6WebEngineWidgets.dll -> Qt6Quick.dll, Qt6QuickWidgets.dll
#   Qt6WebEngineCore.dll    -> Qt6Qml.dll, Qt6Quick.dll, Qt6Network.dll,
#                              Qt6Positioning.dll, Qt6WebChannel.dll
# （Qt6WebEngineCore.dll 203.5 MB 里能搜到这些字符串；文件来自
#   pyside6_webengine-6.12.0.140-cp310-abi3-win_amd64.whl）
# 排除 Python 模块本身不会删掉 DLL（DLL 由 bindepend 依赖分析收），但会把
# QtQuick 关联的插件/qml 目录一起省掉——省的是 1–2 MB，赌的是"白屏"。
# 按"宁可多留也别打出白屏"的原则：**不排除**。
# 同理 QtPositioning（WebEngineCore 的硬依赖）也不排除。

a = Analysis(
    [_ENTRY_PATH],
    pathex=[PY_SRC],                     # laser_desktop / laser_web / robot_model / licensing
    binaries=[],
    datas=datas,
    hiddenimports=hiddenimports,
    hookspath=[],
    # matplotlib 的后端**钉死** Agg：Auto 模式会去猜"这台机器上第一个能 import 的
    # GUI 后端"，而 Windows 构建机是有桌面的、还装了 PySide6 → 猜成 QtAgg，
    # 于是 backend_agg 可能不进来，运行时 matplotlib.use("Agg") 直接 ImportError。
    hooksconfig={"matplotlib": {"backends": "Agg"}},
    runtime_hooks=[],
    excludes=excludes,
    noarchive=False,
    optimize=0,                          # 保留 docstring / assert（排障靠它们）
)

# ---------------------------------------------------------------------------
# 7. 瘦身：QtWebEngine 的 debug 资源与多余 locale
# ---------------------------------------------------------------------------
# TOC 条目是 (目标名, 源路径, 类型)，目标名在第 0 位。
#   * *.debug.pak / *.debug.bin          —— 84.2 MB，DevTools 符号，运行时用不到
#     （qtwebengine_resources.debug.pak、qtwebengine_resources_100p/200p.debug.pak、
#       qtwebengine_devtools_resources.debug.pak、v8_context_snapshot.debug.bin）
#   * qtwebengine_devtools_resources.pak —— 11.6 MB，只有开 --remote-debugging-port
#     才加载；本项目不远程调试，不能为了它多背 11.6 MB。
#   * qtwebengine_locales/*.pak          —— 53 个共 45.8 MB，现场只有中文；
#     留 zh-CN / zh-TW / en-US 三个 = 1.7 MB。
# 保留：qtwebengine_resources.pak、*_100p.pak、*_200p.pak、v8_context_snapshot.bin、
#      icudtl.dat —— 缺任何一个都是白屏或孑进程起不来。
_KEEP_LOCALES = {"zh-CN.pak", "zh-TW.pak", "en-US.pak"}


def _slim(toc, label):
    out = []
    seen = set()
    dropped = 0
    duplicates = 0
    dropped_bytes = 0
    for entry in toc:
        name = str(entry[0]).replace("\\", "/")
        path = str(entry[1]).replace("\\", "/")
        base = os.path.basename(name or path)
        drop = False
        if path.endswith(".debug.pak") or path.endswith(".debug.bin"):
            drop = True
        elif "qtwebengine_devtools_resources" in base:
            drop = True
        elif "qtwebengine_locales" in path and base not in _KEEP_LOCALES:
            drop = True
        if drop:
            dropped += 1
            try:
                dropped_bytes += os.path.getsize(entry[1])
            except OSError:
                pass
            continue
        if name in seen:              # hook 与 collect_data_files 收重了什么
            duplicates += 1          # PyInstaller 也会按目标名去重，这里先算清
            continue
        seen.add(name)
        out.append(entry)
    _log("%s 瘦身：丢 %d 个条目（约 %.1f MB）、去重 %d 个，留 %d 个"
         % (label, dropped, dropped_bytes / 1024.0 / 1024.0, duplicates, len(out)))
    return out


a.datas = _slim(a.datas, "datas")

# ---------------------------------------------------------------------------
# 8. 核对：QtWebEngine 的关键文件必须在，缺了就让构建失败
# ---------------------------------------------------------------------------
# 症状对照（计划 §4.6）：缺 QtWebEngineProcess.exe / resources 时程序照开，
# 只是图区永远空白，控制台/日志里什么都没有 —— 只能靠这里提前拦住。
_REQUIRED = ("QtWebEngineProcess.exe", "qtwebengine_resources.pak",
             "qtwebengine_resources_100p.pak", "qtwebengine_resources_200p.pak",
             "v8_context_snapshot.bin", "icudtl.dat")


def _present_names(toc):
    names = set()
    for entry in toc:
        names.add(os.path.basename(str(entry[0]).replace("\\", "/")))
    return names


def _ensure_qt_conf(datas_toc):
    """``PySide6/qt.conf`` 必须在。

    它是孑进程 ``QtWebEngineProcess.exe`` 找 ``Qt6*.dll`` 的指路牌（相对路径
    ``Prefix = .``）。Qt ≥ 6.3 的 wheel 里**不存在**这个文件，是 PyInstaller 在
    构建时现场生成的；万一哪天 hook 不生成、或本机的 Qt 自带了却没收进来，
    症状同样是白屏。这里缺了就自己写一份（内容与 hook 生成的等价）。
    """
    if "qt.conf" in _present_names(datas_toc):
        return datas_toc
    path = os.path.join(WORK_DIR, "qt.conf")
    with open(path, "w", encoding="utf-8") as fp:
        fp.write("[Paths]\nPrefix = .\n")
    _log("qt.conf 缺（hook 没生成），已自行写入：%s" % path)
    return list(datas_toc) + [(path, "PySide6")]


def _missing_webengine(datas_toc, binaries_toc):
    """还缺哪些 QtWebEngine 关键文件（空列表 = 齐了）。"""
    have = _present_names(datas_toc) | _present_names(binaries_toc)
    missing = [name for name in _REQUIRED if name not in have]
    locale_have = any("qtwebengine_locales/zh-CN.pak" in str(e[0]).replace("\\", "/")
                      for e in datas_toc)
    if not locale_have:
        missing.append("translations/qtwebengine_locales/zh-CN.pak")
    return missing


def _ensure_webengine(datas_toc, binaries_toc):
    missing = _missing_webengine(datas_toc, binaries_toc)
    if not missing:
        _log("QtWebEngine 关键文件齐全：%s" % ", ".join(_REQUIRED))
        return datas_toc

    _log("!! hook 没收到：%s —— 尝试从已安装的 Qt 补齐" % ", ".join(missing))
    try:
        from PySide6.QtCore import QLibraryInfo

        def _qt_path(kind):
            try:                              # Qt6 新 API
                return QLibraryInfo.path(getattr(QLibraryInfo.LibraryPath, kind))
            except (AttributeError, TypeError):   # Qt5 老 API
                return QLibraryInfo.location(getattr(QLibraryInfo, kind))

        exe_dir = _qt_path("LibraryExecutablesPath")
        data_dir = _qt_path("DataPath")
        trans_dir = _qt_path("TranslationsPath")
    except Exception as error:                # noqa: BLE001 - 排障用
        raise SystemExit(
            "[desktop.spec] QtWebEngine 文件缺失且无法通过 Qt 自省补齐：%s\n"
            "  最常见原因：venv 里没有 PySide6-WebEngine（6.12 起 QtWebEngine 的\n"
            "  Python 绑定不再随 PySide6-Addons 发布）。\n"
            "  修：pip install \"PySide6-WebEngine>=6.12,<7\" 后重来。" % error)

    # 补什么、从哪补。源路径必须是**真实存在的目录**：把一个不存在的目录
    # 塞进 TOC，PyInstaller 只会在最后关头报一句看不懂的错，或者干脆打出
    # 一个少了资源的包。所以每个候选源先 stat 一下。
    plan = []                                 # (源路径, 目标目录, 类型, 说明)
    if "QtWebEngineProcess.exe" in missing:
        plan.append((os.path.join(exe_dir, "QtWebEngineProcess.exe"),
                     "PySide6", "binary", "孑进程"))
    _resource_names = ("qtwebengine_resources.pak", "qtwebengine_resources_100p.pak",
                       "qtwebengine_resources_200p.pak", "v8_context_snapshot.bin",
                       "icudtl.dat")
    if any(name in missing for name in _resource_names):
        plan.append((os.path.join(data_dir, "resources"),
                     "PySide6/resources", "data", "resources/"))
    if "translations/qtwebengine_locales/zh-CN.pak" in missing:
        plan.append((os.path.join(trans_dir, "qtwebengine_locales"),
                     "PySide6/translations/qtwebengine_locales", "data",
                     "qtwebengine_locales/"))

    gone = [src for src, _dest, _kind, _what in plan if not os.path.exists(src)]
    if gone:
        raise SystemExit(
            "[desktop.spec] QtWebEngine 的关键文件在这台机上根本找不到：\n  %s\n"
            "  Qt 自省到的位置：exe=%s data=%s translations=%s\n"
            "  说明：装的是不含 QtWebEngine 运行时的 PySide6（Linux wheel 就是\n"
            "  这样，Windows 上则是漏装了 PySide6-WebEngine）。\n"
            "  Windows 构建机上的修法：pip install \"PySide6-WebEngine>=6.12,<7\""
            % ("\n  ".join(gone), exe_dir, data_dir, trans_dir))

    for src, dest, kind, what in plan:
        if kind == "binary":
            binaries_toc = list(binaries_toc) + [(src, dest)]
        else:
            datas_toc = list(datas_toc) + [(src, dest)]
        _log("  补齐 %s <- %s" % (what, src))

    datas_toc = _slim(datas_toc, "datas（补齐后）")
    still = _missing_webengine(datas_toc, binaries_toc)
    if still:
        raise SystemExit(
            "[desktop.spec] 补齐后仍缺：%s\n"
            "  PySide6 装的位置：%s\n"
            "  这是会打出白屏包的那种缺失，构建到此为止。"
            % (", ".join(still), os.path.dirname(exe_dir)))
    _log("已从 %s 补齐 QtWebEngine 文件" % os.path.dirname(exe_dir))
    return datas_toc


a.datas = _ensure_qt_conf(a.datas)
a.datas = _ensure_webengine(a.datas, a.binaries)
a.datas = _ensure_qt_conf(a.datas)

# ---------------------------------------------------------------------------
# 9. EXE / COLLECT
# ---------------------------------------------------------------------------
# 版本资源：Windows 的"属性 → 详细信息"里能看到公司/产品/版本，也顺带让
# 杀软少看一眼"没有任何元数据的 exe"。写成文件是因为 PyInstaller 只吃文件路径。
_VERSION_FILE = os.path.join(WORK_DIR, "version_info.txt")
# filevers 要 4 段整数：从 APP_VERSION 推出来，别在这里再写一遍版本号。
_VER_PARTS = tuple(int(part) for part in APP_VERSION.split(".")) + (0, 0, 0)
with open(_VERSION_FILE, "w", encoding="utf-8") as fp:
    fp.write(
        "# UTF-8\n"
        "VSVersionInfo(\n"
        "  ffi=FixedFileInfo(filevers=%(v4)r, prodvers=%(v4)r,\n"
        "                   mask=0x3f, flags=0x0, OS=0x40004, fileType=0x1,\n"
        "                   subtype=0x0, date=(0, 0)),\n"
        "  kids=[\n"
        "    StringFileInfo([\n"
        "      StringTable('040904B0', [\n"
        "        StringStruct('CompanyName', '%(publisher)s'),\n"
        "        StringStruct('FileDescription', '标定路点工具（桌面版）'),\n"
        "        StringStruct('FileVersion', '%(version)s'),\n"
        "        StringStruct('InternalName', '%(name)s'),\n"
        "        StringStruct('LegalCopyright', 'Copyright (c) 2026'),\n"
        "        StringStruct('OriginalFilename', '%(name)s.exe'),\n"
        "        StringStruct('ProductName', '标定路点工具'),\n"
        "        StringStruct('ProductVersion', '%(version)s')])\n"
        "    ]),\n"
        "    VarFileInfo([VarStruct('Translation', [1033, 1200])])\n"
        "  ]\n"
        ")\n" % {"publisher": APP_PUBLISHER, "version": APP_VERSION,
                 "name": APP_NAME, "v4": _VER_PARTS[:4]})

# 图标是可选的：仓库里现在没有 .ico，没有就不设（PyInstaller 会报错说找不到）。
_ICON = os.path.join(REPO_ROOT, "packaging", "app.ico")
_ICON = _ICON if os.path.isfile(_ICON) else None
if _ICON is None:
    _log("未提供 packaging/app.ico，用 PyInstaller 默认图标")

pyz = PYZ(a.pure)

exe = EXE(
    pyz,
    a.scripts,
    exclude_binaries=True,               # onedir：dll 与数据都交给 COLLECT
    # 钉死 _internal（PyInstaller 6 的默认值，写出来是给 verify_dist.py 与
    # 文档一个不随版本变的名字；改成 '.' 会退回旧的"dll 全铺在根目录"布局）。
    contents_directory="_internal",
    name=APP_NAME,
    console=False,                      # GUI 子系统：不弹黑框
    debug=False,
    strip=False,                        # Windows 上 strip 不支持（Linux 才用）
    upx=False,                          # --noupx：UPX 压缩的 PE 段是杀软启发式高危特征
    upx_exclude=[],
    runtime_tmpdir=None,
    icon=_ICON,
    version=_VERSION_FILE,
    disable_windowed_traceback=False,   # 崩溃时给一个可复制的错误框，比"闪退"强
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
)

coll = COLLECT(
    exe,
    a.binaries,
    a.datas,
    strip=False,
    upx=False,                          # Chromium 的 dll 尤其不能被 UPX 碰
    upx_exclude=[],
    name=APP_NAME,
)

_log("完成。产物：dist/%s/%s.exe（%d 个数据文件）" % (APP_NAME, APP_NAME, len(a.datas)))
