<#
.SYNOPSIS
    在 Windows 构建机上把桌面版打成安装包（PyInstaller onedir + Inno Setup 6）。

.DESCRIPTION
    八个步骤，任何一步失败就停，并打印"人话"错误（不是 PowerShell 的堆栈）。
    每一步都打印**实际值**（版本号、路径、体积），方便把整段输出贴回来复盘。

    1. 检查 Python（3.13 x64；打印实际值与解释器路径）
    2. 建/复用 venv，装 requirements-desktop.txt + pyinstaller==6.16.*，
       并实测 PySide6 的 QtWebEngine 能 import（6.12 起它可能不在 Addons 里）
    3. 把 plotly.min.js 落到 data\assets\plotly\（离线画图的资产）
    4. pytest python -q —— 红的代码不许打包
    5. pyinstaller packaging\desktop.spec --noconfirm（onedir，无 UPX）
    6. packaging\verify_dist.py —— 逐项查产物（缺 WebEngine 资源就当场失败）
    7. 找 ISCC.exe，打 dist\setup\LaserCalibration-setup-<版本>.exe
    8. 打印产物路径与体积

    **为什么只能在 Windows 上跑**：PyInstaller 不是交叉编译器，它复制的是当前
    正在运行的解释器连同 .pyd/.dll 与平台启动器。Linux 上跑只会得到一堆 ELF。

.PARAMETER PythonExe
    用哪个 Python 建 venv（默认自动找 py -3.13 → python）。

.PARAMETER VenvPath
    venv 放哪（默认 build\venv-desktop；build\ 已被 .gitignore 忽略）。

.PARAMETER SkipTests
    跳过第 4 步（**只在排障时用**；跳过就意味着可能拿红代码打包）。

.PARAMETER SkipInstaller
    只做到第 6 步，不打安装包。

.PARAMETER Clean
    给 PyInstaller 传 --clean，并先删掉旧的 dist\LaserCalibration。

.PARAMETER SkipPythonCheck
    不强制 3.13 x64（**不推荐**：包里的解释器版本会与目标机不一致）。

.EXAMPLE
    powershell -ExecutionPolicy Bypass -File packaging\build_windows.ps1

.EXAMPLE
    .\packaging\build_windows.ps1 -Clean -SkipInstaller
#>
[CmdletBinding()]
param(
    [string] $PythonExe = "",
    [string] $VenvPath = "",
    [switch] $SkipTests,
    [switch] $SkipInstaller,
    [switch] $Clean,
    [switch] $SkipPythonCheck
)

$ErrorActionPreference = 'Stop'

# ---------------------------------------------------------------------------
# 0. 环境准备
# ---------------------------------------------------------------------------
# $PSScriptRoot = 本脚本所在目录（packaging\），仓库根是它上一级。
if (-not $PSScriptRoot) {
    Write-Host "无法确定脚本所在目录（请用 -File 方式调用本脚本）。" -ForegroundColor Red
    exit 1
}
$PackagingDir = $PSScriptRoot
$RepoRoot = Split-Path -Parent $PackagingDir
$PySrc = Join-Path $RepoRoot 'python'
$SpecPath = Join-Path $PackagingDir 'desktop.spec'
$VerifyScript = Join-Path $PackagingDir 'verify_dist.py'
$IssPath = Join-Path $PackagingDir 'setup.iss'
$DistDir = Join-Path $RepoRoot 'dist\LaserCalibration'
$SetupDir = Join-Path $RepoRoot 'dist\setup'
if (-not $VenvPath) {
    $VenvPath = Join-Path $RepoRoot 'build\venv-desktop'
} elseif (-not [System.IO.Path]::IsPathRooted($VenvPath)) {
    # 相对路径按仓库根解析（不是按当前所在目录），免得在 packaging\ 里跑出第二份 venv。
    $VenvPath = Join-Path $RepoRoot $VenvPath
}

$TotalSteps = 8
$script:StepNo = 0

function Write-Step {
    param([string] $Title)
    $script:StepNo = $script:StepNo + 1
    Write-Host ""
    Write-Host ("[{0}/{1}] {2}" -f $script:StepNo, $TotalSteps, $Title) -ForegroundColor Cyan
    Write-Host ("-" * 74) -ForegroundColor DarkGray
}

function Write-Hint {
    param([string] $Text)
    Write-Host ("  · " + $Text) -ForegroundColor DarkGray
}

function Fail {
    param(
        [string] $Message,
        [string[]] $Hints = @()
    )
    Write-Host ""
    Write-Host ("[失败] " + $Message) -ForegroundColor Red
    foreach ($hint in $Hints) {
        Write-Host ("       " + $hint) -ForegroundColor Yellow
    }
    Write-Host ""
    Write-Host "构建已停止（半成品不算产物；改完再跑一次）。" -ForegroundColor Red
    exit 1
}

function Invoke-Native {
    <#
      跑一个原生命令并检查退出码。PowerShell 默认**不会**因为外部 exe 返回非 0
      而报错（$ErrorActionPreference 管不到它），所以每个外部命令都必须自己看
      $LASTEXITCODE。这里统一收口。
    #>
    param(
        [string] $What,
        [scriptblock] $Body,
        [string[]] $Hints = @()
    )
    try {
        & $Body
    } catch {
        # 命令不存在 / 路径写错这类问题会抛终止异常，转成人话（不要甩堆栈）。
        Fail -Message ("{0} 抛异常：{1}" -f $What, $_.Exception.Message) -Hints $Hints
    }
    if ($LASTEXITCODE -ne 0) {
        Fail -Message ("{0} 失败（退出码 {1}）" -f $What, $LASTEXITCODE) -Hints $Hints
    }
}

function Invoke-Py {
    <#
      用"某个 python"执行参数。$Runner 可能带前缀（py -3.13），
      所以分两段拼接：& $exe @prefix @Arguments。
    #>
    param(
        [string[]] $Runner,
        [string[]] $Arguments
    )
    $exe = $Runner[0]
    $prefix = @()
    if ($Runner.Count -gt 1) { $prefix = $Runner[1..($Runner.Count - 1)] }
    & $exe @prefix @Arguments
}

function Get-PythonRunner {
    <# 找可用的 Python：显式参数 > py -3.13 > python。返回数组（含前缀参数）。 #>
    if ($PythonExe) { return @($PythonExe) }
    $launcher = Get-Command py.exe -ErrorAction SilentlyContinue
    if ($launcher) { return @($launcher.Source, '-3.13') }
    $python = Get-Command python.exe -ErrorAction SilentlyContinue
    if ($python) { return @($python.Source) }
    Fail -Message "这台机器上找不到 Python。" -Hints @(
        "装 Python 3.13 x64（https://www.python.org/downloads/windows/），",
        "安装时勾选 'Add python.exe to PATH'，然后重开一个终端再跑一次。",
        "或直接指定：-PythonExe C:\Users\you\AppData\Local\Python\pythoncore-3.13-64\python.exe")
}

function Format-MB {
    param([double] $Bytes)
    return ("{0:N1} MB" -f ($Bytes / 1MB))
}

function Get-DirSizeBytes {
    param([string] $Path)
    if (-not (Test-Path -LiteralPath $Path)) { return 0 }
    $measured = Get-ChildItem -LiteralPath $Path -Recurse -File -ErrorAction SilentlyContinue |
                Measure-Object -Property Length -Sum
    if (-not $measured.Sum) { return 0 }
    return [double] $measured.Sum
}

# 控制台按 UTF-8 输出（中文路径/中文错误信息在 cp936 下会变乱码）。改不动也不致命。
try {
    [Console]::OutputEncoding = [System.Text.Encoding]::UTF8
    $env:PYTHONIOENCODING = 'utf-8'
    $env:PYTHONUTF8 = '1'
    $env:PIP_DISABLE_PIP_VERSION_CHECK = '1'
} catch { }

Write-Host ""
Write-Host "桌面版 Windows 构建" -ForegroundColor Green
Write-Host ("  仓库根   : " + $RepoRoot)
Write-Host ("  打包脚本 : " + $SpecPath)
Write-Host ("  venv     : " + $VenvPath)
if ($RepoRoot.Length -gt 110) {
    Write-Hint ("警告：仓库路径很长（{0} 字符）。PyInstaller 的中间文件会再套 3–4 层，" -f $RepoRoot.Length)
    Write-Hint "总长可能超过 260；建议把仓库放到 C:\build\ 下构建。"
}

# 版本号写在两处（desktop.spec 的 APP_VERSION 决定 exe 的资源版本，
# setup.iss 的 AppVersion 决定安装包文件名与卸载项）。这里读出来比对：
# 不一致就当场失败——"exe 显示 0.9.0、安装包叫 0.8.0"这种事故现场没法解释。
# 不用 ISCC 的 /DAppVersion 传：ISPP 会把它当表达式解析，0.9.0 里的两个点会报错。
$AppVersion = '0.9.0'
$IssVersion = ''
$specText = Get-Content -LiteralPath $SpecPath -Raw -Encoding UTF8
if ($specText -match 'APP_VERSION\s*=\s*"([0-9]+\.[0-9]+\.[0-9]+)"') {
    $AppVersion = $Matches[1]
}
$issText = Get-Content -LiteralPath $IssPath -Raw -Encoding UTF8
if ($issText -match '(?m)^\s*#define\s+AppVersion\s+"([0-9]+\.[0-9]+\.[0-9]+)"') {
    $IssVersion = $Matches[1]
}
Write-Host ("  版本     : " + $AppVersion + "（desktop.spec 的 APP_VERSION）")
if ($IssVersion -and ($IssVersion -ne $AppVersion)) {
    Fail -Message ("版本号不一致：desktop.spec 是 " + $AppVersion + "，setup.iss 是 " + $IssVersion + "。") -Hints @(
        "两处一起改（升级时最容易漏的就是安装包那一处）。",
        ("desktop.spec: " + $SpecPath),
        ("setup.iss   : " + $IssPath))
}

# ---------------------------------------------------------------------------
# 1. Python：3.13 + x64
# ---------------------------------------------------------------------------
Write-Step "检查 Python（要求 3.13 x64：包里的解释器版本要与目标机一致）"
$Runner = Get-PythonRunner
$probeCode = "import sys, struct, platform; print(platform.python_version()); print(struct.calcsize('P') * 8); print(sys.executable)"
$probe = Invoke-Py -Runner $Runner -Arguments @('-c', $probeCode)
if ($LASTEXITCODE -ne 0) {
    Fail -Message ("探测 Python 版本失败（{0}）。" -f ($Runner -join ' ')) -Hints @(
        "如果是 Microsoft Store 的 python.exe，它会拒绝这种调用；换 python.org 的安装包。")
}
$probeLines = @($probe | Where-Object { $_ -and $_.Trim() -ne '' })
if ($probeLines.Count -lt 3) {
    Fail -Message "读不到 Python 的版本信息（输出不完整）。" -Hints @($probeLines -join ' | ')
}
$PyVersion = $probeLines[0].Trim()
$PyBits = $probeLines[1].Trim()
$PyExe = $probeLines[2].Trim()
Write-Host ("  Python   : " + $PyVersion)
Write-Host ("  位数     : " + $PyBits + " bit")
Write-Host ("  解释器   : " + $PyExe)
if (-not $SkipPythonCheck) {
    if ($PyBits -ne '64') {
        Fail -Message ("解释器是 " + $PyBits + " 位的，必须 64 位。") -Hints @(
            "QtWebEngine 只有 64 位 wheel；32 位 Python 打出来的包一开图就白屏。")
    }
    if (-not ($PyVersion -like '3.13.*')) {
        Fail -Message ("解释器是 Python " + $PyVersion + "，要求 3.13.x。") -Hints @(
            "PyInstaller 会把**这个**解释器打进包里，版本必须与目标机一致",
            "（目标机实测 3.13.17 x64）。装 Python 3.13 后重跑；",
            "确实要用别的版本：加 -SkipPythonCheck（不推荐）。")
    }
    Write-Host "  [OK] 3.13 x64，与目标机一致" -ForegroundColor Green
}

# ---------------------------------------------------------------------------
# 2. venv + 依赖
# ---------------------------------------------------------------------------
Write-Step "准备 venv 并安装依赖（requirements-desktop.txt + pyinstaller 6.16.*）"
$VenvPython = Join-Path $VenvPath 'Scripts\python.exe'
if (Test-Path -LiteralPath $VenvPython) {
    Write-Host ("  复用已有 venv：" + $VenvPath)
} else {
    Write-Host ("  新建 venv：" + $VenvPath)
    Invoke-Native -What "创建 venv" -Body { Invoke-Py -Runner $Runner -Arguments @('-m', 'venv', $VenvPath) } -Hints @(
        "如果提示 'ensurepip is not available'：这个 Python 缺 venv 组件，",
        "用 python.org 的官方安装包重装一遍。")
    if (-not (Test-Path -LiteralPath $VenvPython)) {
        Fail -Message ("venv 建完了却没有解释器：" + $VenvPython) -Hints @(
            "venv 可能是半成品（磁盘满、杀软拦了文件写入）；删掉 build\venv-desktop 重跑。")
    }
}
Invoke-Native -What "pip 升级" -Body { & $VenvPython -m pip install --upgrade pip --quiet }
$reqs = Join-Path $RepoRoot 'requirements-desktop.txt'
Write-Host ("  安装：" + $reqs)
Invoke-Native -What "pip install -r requirements-desktop.txt" -Body {
    & $VenvPython -m pip install -r $reqs --quiet
} -Hints @("内网构建机先配 pip 源：pip config set global.index-url <内网源>")
Write-Host "  安装：pyinstaller==6.16.*"
Invoke-Native -What "pip install pyinstaller" -Body {
    & $VenvPython -m pip install 'pyinstaller==6.16.*' --quiet
} -Hints @("6.16 是确认支持 Python 3.13 + PySide6 6.12 的版本线。")

# PySide6 6.12 起 QtWebEngine 的 Python 绑定可能不在 PySide6-Addons 里了。
# requirements-desktop.txt 在 S7 里不许改，所以这里**实测** import，缺了就
# 补那一条 wheel 并把命令打印出来——比让人对着一句 ImportError 猜半天强。
Write-Host "  实测 QtWebEngine 能否 import ……"
& $VenvPython -c "from PySide6.QtWebEngineWidgets import QWebEngineView; from PySide6.QtWebEngineCore import QWebEngineSettings; import PySide6; print('  PySide6', PySide6.__version__, 'QtWebEngine OK')"
if ($LASTEXITCODE -ne 0) {
    Write-Host "  缺 QtWebEngine，补装 PySide6-WebEngine ……" -ForegroundColor Yellow
    & $VenvPython -m pip install 'PySide6-WebEngine>=6.12,<7' --quiet
    if ($LASTEXITCODE -ne 0) {
        Fail -Message "装不上 PySide6-WebEngine。" -Hints @(
            "离线环境先 pip download PySide6-WebEngine==6.12.0.140 带过来再 install。")
    }
    & $VenvPython -c "from PySide6.QtWebEngineWidgets import QWebEngineView; print('  QtWebEngine OK（补装后）')"
    if ($LASTEXITCODE -ne 0) {
        Fail -Message "补装后仍然 import 不了 QtWebEngine。" -Hints @(
            "查一下是不是有系统级 PySide6 抢了路径（venv 里不该混入）。")
    }
}
Write-Host "  [OK] QtWebEngine 可用" -ForegroundColor Green

# ---------------------------------------------------------------------------
# 3. 离线资产：plotly.min.js
# ---------------------------------------------------------------------------
Write-Step "准备离线资产 plotly.min.js（data\assets\plotly\）"
$assetDir = Join-Path $RepoRoot 'data\assets\plotly'
$assetJs = Join-Path $assetDir 'plotly.min.js'
if (Test-Path -LiteralPath $assetJs) {
    Write-Host ("  已有：" + $assetJs)
} else {
    New-Item -ItemType Directory -Force -Path $assetDir | Out-Null
    # 路径当参数传给 Python（sys.argv[1]），不拼进代码字符串：仓库路径里有空格/
    # 引号时不会把 -c 那段代码打断。
    Invoke-Native -What "从 plotly 包复制 plotly.min.js" -Body {
        & $VenvPython -c "import plotly, shutil, pathlib, sys; src = pathlib.Path(plotly.__file__).parent / 'package_data' / 'plotly.min.js'; dst = pathlib.Path(sys.argv[1]); dst.mkdir(parents=True, exist_ok=True); shutil.copy2(str(src), str(dst / 'plotly.min.js')); print('  plotly', plotly.__version__, '->', dst / 'plotly.min.js')" $assetDir
    }
}
if (-not (Test-Path -LiteralPath $assetJs)) {
    Fail -Message "没有 plotly.min.js。" -Hints @(
        "离线画图全靠它；先确认 plotly 装上了：& $VenvPython -m pip show plotly")
}
$assetSize = (Get-Item -LiteralPath $assetJs).Length
if ($assetSize -lt 3500000) {
    Fail -Message ("plotly.min.js 只有 " + (Format-MB $assetSize) + "，明显不完整。") -Hints @(
        "正常约 4.6 MB；被 Git LFS 占位符替换过、或复制中断都会变成几十 KB。")
}
Write-Host ("  [OK] plotly.min.js " + (Format-MB $assetSize)) -ForegroundColor Green

# ---------------------------------------------------------------------------
# 4. 测试
# ---------------------------------------------------------------------------
Write-Step "跑测试（pytest python -q；红的代码不许打包）"
if ($SkipTests) {
    Write-Host "  已按 -SkipTests 跳过 —— 这一版没有测试背书。" -ForegroundColor Yellow
} else {
    $env:PYTHONPATH = $PySrc
    $pytestCode = 0
    Push-Location $RepoRoot
    try {
        & $VenvPython -m pytest python -q
        $pytestCode = $LASTEXITCODE
    } finally {
        Pop-Location
        Remove-Item Env:\PYTHONPATH -ErrorAction SilentlyContinue
    }
    if ($pytestCode -ne 0) {
        Fail -Message ("pytest 退出码 " + $pytestCode + "（照上面最后几行改代码）。") -Hints @(
            "不许拿红的代码打包：打出来的 exe 修不了，只能重装。",
            "确认是环境问题而不是代码问题，再考虑 -SkipTests。")
    }
    Write-Host "  [OK] 测试全绿" -ForegroundColor Green
}

# ---------------------------------------------------------------------------
# 5. PyInstaller
# ---------------------------------------------------------------------------
Write-Step "PyInstaller 打包（onedir，--noupx）"
if ($Clean -and (Test-Path -LiteralPath $DistDir)) {
    Write-Host ("  删除旧产物：" + $DistDir)
    Remove-Item -LiteralPath $DistDir -Recurse -Force
}
$pyiArgs = @('-m', 'PyInstaller', $SpecPath, '--noconfirm', '--noupx',
             '--distpath', (Join-Path $RepoRoot 'dist'),
             '--workpath', (Join-Path $RepoRoot 'build\desktop'))
if ($Clean) { $pyiArgs += '--clean' }
# 只用 venv 的解释器，不"激活"环境：PATH 上就算有系统级 PySide6 也进不来。
$env:PYTHONPATH = $PySrc
Push-Location $RepoRoot
try {
    Invoke-Native -What "PyInstaller" -Body { Invoke-Py -Runner @($VenvPython) -Arguments $pyiArgs } -Hints @(
        "spec 的报错以 [desktop.spec] 开头，会说清缺哪个文件；",
        "'QtWebEngine 的关键文件在这台机上根本找不到' → 装 PySide6-WebEngine。")
} finally {
    Pop-Location
    Remove-Item Env:\PYTHONPATH -ErrorAction SilentlyContinue
}
$exePath = Join-Path $DistDir 'LaserCalibration.exe'
if (-not (Test-Path -LiteralPath $exePath)) {
    Fail -Message ("PyInstaller 说完成了，但没有 " + $exePath) -Hints @(
        "看 build\desktop\warn-desktop.txt 与 *.log 的最后 30 行；也可能是磁盘满了。")
}
Write-Host ("  [OK] " + $exePath + "（" + (Format-MB (Get-Item -LiteralPath $exePath).Length) + "）") -ForegroundColor Green

# ---------------------------------------------------------------------------
# 6. 产物自检
# ---------------------------------------------------------------------------
Write-Step "产物自检（verify_dist.py：必需文件 / 不该有的东西 / 体积 / --self-check）"
Push-Location $RepoRoot
try {
    Invoke-Native -What "verify_dist.py" -Body {
        & $VenvPython $VerifyScript --dist $DistDir
    } -Hints @(
        "FAIL 项逐条写在上面；改 spec 后重跑，不要手动往 dist 里拷文件。",
        "WARN 只影响体积/观感，不影响能不能跑。")
} finally {
    Pop-Location
}

# ---------------------------------------------------------------------------
# 7. Inno Setup
# ---------------------------------------------------------------------------
Write-Step "打安装包（Inno Setup 6）"
$installerPath = $null
if ($SkipInstaller) {
    Write-Host "  已按 -SkipInstaller 跳过。" -ForegroundColor Yellow
} else {
    $isccPath = $null
    $onPath = Get-Command iscc.exe -ErrorAction SilentlyContinue
    if ($onPath) { $isccPath = $onPath.Source }
    if (-not $isccPath) {
        $roots = @()
        if (${env:ProgramFiles(x86)}) { $roots += ${env:ProgramFiles(x86)} }
        if ($env:ProgramFiles) { $roots += $env:ProgramFiles }
        if ($env:LOCALAPPDATA) { $roots += (Join-Path $env:LOCALAPPDATA 'Programs') }
        foreach ($root in $roots) {
            $candidate = Join-Path $root 'Inno Setup 6\ISCC.exe'
            if (Test-Path -LiteralPath $candidate) { $isccPath = $candidate; break }
        }
    }
    if (-not $isccPath) {
        Fail -Message "找不到 ISCC.exe（Inno Setup 6 的编译器）。" -Hints @(
            "装 Inno Setup 6：https://jrsoftware.org/isdl.php ，装完重跑；",
            "或把它加到 PATH；或只打 exe：-SkipInstaller。",
            "Inno Setup 5 不行（Unicode 与 x64 指令都不一样）。")
    }
    Write-Host ("  ISCC     : " + $isccPath)
    $isccDir = Split-Path -Parent $isccPath

    # 中文语言文件不在官方安装包里（见 setup.iss 文件头）。没有就退回英文向导，
    # 不要因为少一个翻译文件就把构建卡死。
    $chineseIsl = Join-Path $isccDir 'Languages\ChineseSimplified.isl'
    # 只传语言开关这一个 /D（值是 0/1，ISPP 能直接当布尔用；版本号不走这条路，
    # 见上面对两处 #define 的比对）。
    $defineArgs = @()
    if (Test-Path -LiteralPath $chineseIsl) {
        $defineArgs += '/DChineseISL=1'
        Write-Host ("  中文语言 : 有（" + $chineseIsl + "）")
    } else {
        $defineArgs += '/DChineseISL=0'
        Write-Host "  中文语言 : 没有（这一次的安装向导是英文）" -ForegroundColor Yellow
        Write-Hint "要中文向导：从 https://jrsoftware.org/files/istrans/ 下 ChineseSimplified.isl"
        Write-Hint ("放进 " + (Join-Path $isccDir 'Languages') + " 后重跑本脚本。")
    }

    # setup.iss 里有中文，必须是 UTF-8 **带 BOM**：Inno 在没有 BOM 时按系统 ANSI
    # 读脚本，中文会变成乱码（编译能过，界面上全是问号）。
    $issBytes = [System.IO.File]::ReadAllBytes($IssPath)
    $hasBom = ($issBytes.Length -ge 3) -and ($issBytes[0] -eq 0xEF) -and ($issBytes[1] -eq 0xBB) -and ($issBytes[2] -eq 0xBF)
    if (-not $hasBom) {
        Fail -Message "setup.iss 不是 UTF-8 with BOM（中文会变乱码）。" -Hints @(
            "VS Code 打开 → 右下角编码 → Save with Encoding → UTF-8 with BOM。")
    }

    Push-Location $RepoRoot
    try {
        Invoke-Native -What "ISCC" -Body { & $isccPath @defineArgs $IssPath } -Hints @(
            "常见原因：dist\LaserCalibration\ 不存在（先跑第 5 步）；",
            "或装的是 Inno Setup 5.x（这里要 6.x）。")
    } finally {
        Pop-Location
    }
    $installerPath = Join-Path $SetupDir ('LaserCalibration-setup-' + $AppVersion + '.exe')
    if (-not (Test-Path -LiteralPath $installerPath)) {
        $found = @(Get-ChildItem -LiteralPath $SetupDir -Filter '*.exe' -ErrorAction SilentlyContinue)
        if ($found.Count -gt 0) {
            $installerPath = $found[0].FullName
        } else {
            Fail -Message ("ISCC 完成了，但 " + $SetupDir + " 里没有 setup.exe") -Hints @(
                "看 setup.iss 的 OutputDir（现在是 ..\dist\setup）。")
        }
    }
    Write-Host ("  [OK] " + $installerPath) -ForegroundColor Green

    # 有了安装包再把它的体积也验一遍（verify_dist.py --installer）。
    Invoke-Native -What "verify_dist.py --installer（体积复核）" -Body {
        & $VenvPython $VerifyScript --dist $DistDir --installer $installerPath --skip-self-check
    }
}

# ---------------------------------------------------------------------------
# 8. 汇报
# ---------------------------------------------------------------------------
Write-Step "产物"
$distBytes = Get-DirSizeBytes $DistDir
Write-Host ("  解压目录 : " + $DistDir)
Write-Host ("             体积 " + (Format-MB $distBytes) + "（预期 395–490 MB）")
Write-Host ("  主程序   : " + $exePath + "（" + (Format-MB (Get-Item -LiteralPath $exePath).Length) + "）")
if ($installerPath -and (Test-Path -LiteralPath $installerPath)) {
    $instSize = (Get-Item -LiteralPath $installerPath).Length
    Write-Host ("  安装包   : " + $installerPath + "（" + (Format-MB $instSize) + "，预期 155–200 MB）")
    Write-Host ("  安装包哈希: Get-FileHash -Algorithm SHA256 '" + $installerPath + "'")
} else {
    Write-Host "  安装包   : （本次没打）"
}
Write-Host ""
Write-Host "下一步（S7 验收，必须在**干净机**上做）：" -ForegroundColor Green
Write-Host "  1. 干净 Win10 x64（无 Python、无 VC++ redist）双击安装包 → 桌面图标 → 打开"
Write-Host "  2. 跑一次 50 点解算 → 导出 GP；四页的图都要出（白屏看计划 §4.6）"
Write-Host "  3. 任务管理器里 QtWebEngineProcess ≤ 2；netstat -ano 无本进程监听"
Write-Host "  4. 卸载 → 确认按选择保留/删除了 %LOCALAPPDATA%\LaserWaypoint"
Write-Host ""
Write-Host "构建完成。" -ForegroundColor Green
exit 0
