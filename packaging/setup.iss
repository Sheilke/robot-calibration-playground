; ===========================================================================
; Inno Setup 6 脚本：把 dist\LaserCalibration\ 打成 setup.exe
;
; 编译（Windows 构建机）：
;     "C:\Program Files (x86)\Inno Setup 6\ISCC.exe" packaging\setup.iss
; 或者直接跑 packaging\build_windows.ps1（它会找 ISCC 并带上 /D 参数）。
;
; 安装形态（三个"为什么"）
; ------------------------
; 1. **免管理员**：PrivilegesRequired=lowest + 装到
;    {localappdata}\Programs\LaserCalibration。现场 PC 多为受控账户，弹 UAC
;    要 IT 到场；而且程序的可写数据（runs/、缓存）本来就落在
;    %LOCALAPPDATA%，装在用户目录里前后一致。
; 2. **AppId 固定**：升级时 Inno 认得出"这是同一个产品"，直接覆盖安装并沿用
;    上次的目录；AppId 一变，老版本就成了删不掉的野鬼。
; 3. **不写自启动**：首版没有托盘常驻需求（计划 §8.1），也就不碰
;    Run/RunOnce 键——现场 IT 对这类写入最敏感。
; ===========================================================================

; 版本号：这里与 packaging/desktop.spec 的 APP_VERSION 必须一致。
; build_windows.ps1 会把两处读出来比对，不一致直接失败（exe 显示 0.9.0、
; 安装包叫 0.8.0 这种事故，现场很难解释）。升级时两处一起改。
; （不用 ISCC 的 /DAppVersion=... 传：ISPP 会把它当表达式解析，0.9.0 里两个点
;   会直接报语法错；字符串要套一层引号再转义，命令行上太容易出错。）
#define AppVersion "0.9.0"

; 中文语言文件**不在** Inno Setup 官方安装包里（官方只带英/法/德/日等，中文要
; 从 https://jrsoftware.org/files/istrans/ 下 ChineseSimplified.isl 放进
; Inno Setup 6\Languages\）。build_windows.ps1 会检查这个文件，
; 没有就用 /DChineseISL=0 关掉中文，只装英文向导——不会因为缺一个翻译文件
; 就编译失败。手工编译时若报"无法读取 Languages\ChineseSimplified.isl"，
; 同样加 /DChineseISL=0。
; **默认关**。理由：默认开会让"手工跑一次 ISCC"直接编译失败（报"无法读取
; Languages\ChineseSimplified.isl"），而那种失败看起来像 .iss 写错了，实际只是
; 少一个可选翻译文件。默认关 = 缺文件也能出英文向导包；build_windows.ps1 检查到
; 有这个文件时自己传 /DChineseISL=1 打开中文。
#ifndef ChineseISL
  #define ChineseISL 0
#endif

#define AppName "标定路点工具"
#define AppShortName "LaserCalibration"
#define AppPublisher "3S Robotics"
#define AppExeName "LaserCalibration.exe"

[Setup]
; AppId 是"产品身份证"：固定不变才能覆盖升级（Inno 用它找已安装的旧版本）。
; 两侧的 {{ 是 Inno 的转义，最终写入注册表的是一个花括号的 GUID。
AppId={{B7C4F0E2-9A31-4F6D-8E52-3A17C0D9B4A1}
AppName={#AppName}
AppVersion={#AppVersion}
AppVerName={#AppName} {#AppVersion}
AppPublisher={#AppPublisher}
; 文件属性里的「文件版本」必须是 x.y.z.w 四段（Inno 对这一项比
; AppVersion 严格），所以把 0.9.0 补成 0.9.0.0。
VersionInfoVersion={#AppVersion}.0
VersionInfoDescription={#AppName} 安装程序
VersionInfoCompany={#AppPublisher}
; 免管理员安装到用户目录（见文件头第 1 条）。
PrivilegesRequired=lowest
; 装到 %LOCALAPPDATA%\Programs\LaserCalibration（{localappdata} 在 lowest 下
; 就是当前用户；不要用 {autopf}/{pf}，那需要管理员）。
DefaultDirName={localappdata}\Programs\{#AppShortName}
; 开始菜单组：必须项（现场培训文档里写的是"开始菜单里点这个"）。
DefaultGroupName={#AppName}
DisableProgramGroupPage=yes
; 只允许 64 位 x64：QtWebEngine 没有 32 位 wheel，装到 32 位机上只会白屏。
; （Inno ≥ 6.3 支持更准确的 x64compatible——它同时覆盖 ARM64 上的 x64 模拟；
;   为了兼容 6.0–6.2 的编译器，这里用老写法 x64，语义在这台机上完全一样。）
ArchitecturesAllowed=x64
ArchitecturesInstallIn64BitMode=x64
; Windows 10 及以上：Qt 6.x 的最低要求，7/8.1 上没有可用的 Qt6 运行时。
MinVersion=10.0
; 安装包输出到 dist\setup\LaserCalibration-setup-<版本>.exe（与 dist\ 并列）。
OutputDir=..\dist\setup
OutputBaseFilename={#AppShortName}-setup-{#AppVersion}
; lzma2/max + solid：体积按计划 §5.4 的 155–200 MB 目标；solid 让相同 dll 只压一次。
Compression=lzma2/max
SolidCompression=yes
WizardStyle=modern
; 卸载列表里显示名字与图标（图标取自 exe 本身，没有单独的 .ico 也不影响）。
UninstallDisplayName={#AppName}
UninstallDisplayIcon={app}\{#AppExeName}
; 覆盖安装时让用户先关掉正在跑的程序（用 Restart Manager，不需要程序自己
; 注册什么名字）；装完**不**替用户重启程序，避免脚本化安装时冒出窗口。
CloseApplications=yes
RestartApplications=no
; 写一份安装日志到 %TEMP%：现场装不上时让它直接发这个文件，比截图强。
SetupLogging=yes
; 升级时沿用上次的安装目录（配合固定 AppId），不让用户重选。
UsePreviousAppDir=yes
; 允许"不要图标"（开始菜单仍然会建，AllowNoIcons 只关掉图标任务）。
AllowNoIcons=yes
; 不申请任何文件关联：本程序不打开自己的文件类型。
ChangesAssociations=no
; 不加 AppMutex：程序目前**没有**创建具名互斥体（单实例是 QLockFile 文件锁），
; 写一个 AppMutex 只会让 Inno 永远等不到那个名字；关程序靠上面的
; CloseApplications + Restart Manager。（将来加了 QLocalServer/QLockFile 的
; 具名互斥体，再把 AppMutex=Local\{#AppShortName}-desktop 加回来，
; 注意别用 Global\ 前缀——PrivilegesRequired=lowest 下建不出全局对象。）

[Languages]
#if ChineseISL
; 简体中文（需要 Languages\ChineseSimplified.isl，见文件头说明）。
Name: "chinesesimplified"; MessagesFile: "compiler:Languages\ChineseSimplified.isl"
#endif
; 英文作为兜底：万一中文 .isl 缺失，向导至少是能读的英文。
Name: "english"; MessagesFile: "compiler:Default.isl"

[Tasks]
; 桌面快捷方式做成可勾选项（默认勾上：现场第一件事就是双击桌面图标）。
; 开始菜单快捷方式不受这个任务影响，永远创建。
Name: "desktopicon"; Description: "{cm:CreateDesktopIcon}"; GroupDescription: "{cm:AdditionalIcons}"

[Files]
; 整个 PyInstaller onedir 产物原样拷过去：_internal\ 里的 Qt6*.dll、
; QtWebEngineProcess.exe、resources\*.pak、icudtl.dat 一个都不能少
; （verify_dist.py 已经逐项查过）。
;   recursesubdirs/createallsubdirs —— 递归建目录，_internal 有多层；
;   ignoreversion —— 不按文件版本号决定是否覆盖，升级时一律用新文件。
Source: "..\dist\{#AppShortName}\*"; DestDir: "{app}"; Flags: ignoreversion recursesubdirs createallsubdirs

[Icons]
Name: "{group}\{#AppName}"; Filename: "{app}\{#AppExeName}"
Name: "{group}\{cm:UninstallProgram,{#AppName}}"; Filename: "{uninstallexe}"
; {autodesktop} 在 lowest 权限下就是当前用户的桌面（不需要管理员）。
Name: "{autodesktop}\{#AppName}"; Filename: "{app}\{#AppExeName}"; Tasks: desktopicon

[Run]
; 装完直接启动（可取消勾选）；skipifsilent 保证静默安装不弹窗。
Filename: "{app}\{#AppExeName}"; Description: "{cm:LaunchProgram,{#AppName}}"; Flags: nowait postinstall skipifsilent

[Registry]
; 程序用 QSettings 存窗口几何，组织名/应用名是 robot-calibration / laser-desktop
; （laser_desktop/page_layout.py 的 SETTINGS_ORG / SETTINGS_APP），
; 在注册表里就是 HKCU\Software\robot-calibration\laser-desktop。
; uninsdeletekey：卸载时把这个键一起删掉，不留"半套设置"。
Root: HKCU; Subkey: "Software\robot-calibration"; Flags: uninsdeletekey

[Code]
// 卸载时问一句：要不要把运行记录与图表缓存也删掉？
//   * 默认**保留**（MB_DEFBUTTON2 = 默认按钮是"否"）：runs/ 里有用户算出来的
//     scan.json / joints.txt，删掉就再也拿不回来了；
//   * 静默卸载（/SILENT、/VERYSILENT）一律跳过询问并保留，避免脚本化部署
//     把用户数据一起抹掉。
// 目录来源与 laser_desktop/paths.py 对齐：
//   APP_ID = "LaserWaypoint"  →  %LOCALAPPDATA%\LaserWaypoint\{runs,.cache}
//   另附 %LOCALAPPDATA%\robot-calibration（网页版/早期版本用过的名字，有就一起清）。
procedure DeleteUserData(const BaseDir: String);
begin
  if DirExists(BaseDir) then
    DelTree(BaseDir, True, True, True);
end;

function UserDataExists: Boolean;
var
  Base: String;
begin
  Base := ExpandConstant('{localappdata}');
  Result := DirExists(Base + '\LaserWaypoint')
         or DirExists(Base + '\robot-calibration');
end;

procedure CurUninstallStepChanged(CurUninstallStep: TUninstallStep);
var
  Base: String;
  Prompt: String;
begin
  // 放到 usPostUninstall：程序文件已经删完，此时删用户数据不会撞上占用。
  if CurUninstallStep <> usPostUninstall then
    Exit;
  // 静默卸载不问（也不删）：自动化流程里不能出现等待输入的对话框。
  if UninstallSilent then
    Exit;
  if not UserDataExists then
    Exit;

  Base := ExpandConstant('{localappdata}');
  Prompt := '是否同时删除运行记录与图表缓存？' + #13#10 + #13#10
          + Base + '\LaserWaypoint\' + #13#10
          + '　　（runs\ 解算结果与日志、.cache\ 图表缓存）' + #13#10
          + Base + '\robot-calibration\' + #13#10 + #13#10
          + '选“否”（默认）会保留这些文件：重装后还能看到以前的解算结果，'
          + '但会占用磁盘空间。';
  if MsgBox(Prompt, mbConfirmation, MB_YESNO or MB_DEFBUTTON2) = IDYES then
  begin
    DeleteUserData(Base + '\LaserWaypoint');
    DeleteUserData(Base + '\robot-calibration');
  end;
end;
