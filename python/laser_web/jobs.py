# -*- coding: utf-8 -*-
"""后台任务登记表：一条线程串行跑解算，UI 只轮询快照。

设计对应设计文档 §7 的第 1、2、3、7 条：

1. **不阻塞 UI**：``submit`` 只登记任务并返回 id，真正干活的是本模块内部
   唯一的一条工作线程；IK 吃满一个核，因此队列化串行（同一时间只跑一个
   任务），互相抢核会把两者都拖慢；
2. **日志捕获**：任务执行期间把 ``sys.stdout`` 重定向到环形缓冲（最多
   :data:`LOG_LIMIT` 行），于是 ``solve_chain_guarded`` 内部的 ``print``
   也能在界面上看到，等价于终端输出；
3. **取消**：任务持有一个 ``threading.Event``；被调用方在**阶段边界**检查
   （见 :class:`JobCancelled`），本模块在任务返回/抛错后再复核一次标志，
   保证「点了取消就一定显示已取消」；
4. **保留策略**：每次新建 ``runs/<id>/`` 后把超过 :data:`KEEP_RUNS` 个的
   最旧运行目录删掉，磁盘不会被几十次解算的产物撑爆；
5. **订阅（桌面端用，S3 新增）**：``Job.subscribe`` 让工作线程在状态 / 进度 /
   日志变化时就地回调，桌面版因此不必轮询 ``snapshot()``。回调跑在**工作
   线程**里、并且在锁外调用（回调里再 ``snapshot()`` 不会死锁），监听器自己
   抛异常只记一条 warning（带栈）、不拖垮任务。网页版继续用 ``snapshot`` 轮询，
   两条路互不影响（``snapshot`` 的行为一字未改）。

本模块只依赖标准库（``core`` 可以安全 import 它，不会把 UI 库拖进来）。
"""

from __future__ import annotations

import contextlib
import io
import logging
import os
import queue
import secrets
import shutil
import threading
from collections.abc import Callable
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any

_LOGGER = logging.getLogger(__name__)

#: 单任务日志环形缓冲上限（行）。
#: 与桌面版日志控件（``laser_desktop.generate_page.LOG_WIDGET_LIMIT``）一致。
#: S6 之前是 200：界面说明写的是"控件最多留 500 行"，而源头只留 200，
#: 用户会以为"中间那段只是没显示"，其实源头就已经丢了。两边相等时，
#: 能显示多少就是源头留了多少。
LOG_LIMIT = 500
#: ``runs/`` 下保留的运行目录个数（含最新创建的那个）。
KEEP_RUNS = 50
#: 登记表里最多保留多少条**已结束**的任务（未结束的永不淘汰）。
#: 每条任务带着 LOG_LIMIT 行日志与完整 result（几十 KB~几百 KB），不设上限的话
#: 挂机一整天就是在漏内存。超过时在 :meth:`JobRegistry.submit` 里丢最旧的。
MAX_FINISHED = 50
#: 判定“运行目录已经有成果”的产物名（与 ``core.ARTIFACT_SCAN_JSON`` 同名）。
#: ``core`` 反过来 import 本模块，所以常量放在这边，由测试钉住两边一致。
SCAN_ARTIFACT = "scan.json"
#: 取消后保留目录的产物判据（见 ``_discard_partial_run``）。
#: ``iso.json`` 是 ISO 五点页的产物，与 ``scan.json`` 同构（都能喂 ``export_gp``），
#: 所以也能算“这次运行已经有成果”。
RESULT_ARTIFACTS = (SCAN_ARTIFACT, "iso.json")
#: 运行产物目录名（仓库根下，已写入 ``.gitignore``）。
RUNS_DIRNAME = "runs"
#: 覆盖产物目录的环境变量（测试用；不设则 ``<repo>/runs``）。
RUNS_ENV = "LASER_WEB_RUNS"

#: 任务种类的合法取值（与 ``core.run_scan`` 的 ``kind`` 对应）。
KINDS = ("scan", "repair", "iso")
#: 任务状态的合法取值。
STATES = ("pending", "running", "done", "failed", "cancelled")
#: 已结束的状态集合（用于判断是否可以再取消）。
TERMINAL_STATES = ("done", "failed", "cancelled")

_REPO_ROOT = Path(__file__).resolve().parents[2]
_HEX = "0123456789abcdef"


class JobCancelled(RuntimeError):
    """任务被用户取消（阶段边界处抛出，由 :class:`JobRegistry` 归为 cancelled）。

    任务函数（``core.run_scan`` 之类）在阶段边界检查 ``job.cancel.is_set()``
    后抛本异常即可立刻中止；异常消息只用于日志。
    """


def repo_root() -> Path:
    """仓库根（``<repo>/python/laser_web/jobs.py`` 往上两级）。"""
    return _REPO_ROOT


def runs_root(root: str | Path | None = None) -> Path:
    """解析产物根目录：显式参数 > 环境变量 :data:`RUNS_ENV` > ``<repo>/runs``。

    Args:
        root: 显式指定（测试/多实例用）；``None`` 表示按环境变量与默认值解析。

    Returns:
        Path: 绝对路径（不保证已存在）。
    """
    if root is not None:
        return Path(root).expanduser().resolve()
    override = os.environ.get(RUNS_ENV)
    if override:
        return Path(override).expanduser().resolve()
    return _REPO_ROOT / RUNS_DIRNAME


def new_run_id(now: datetime | None = None) -> str:
    """生成运行 id：``20261116-173045-a1b2``（时间戳 + 4 位随机十六进制）。

    Args:
        now: 时间戳来源；``None`` 表示当前本地时间（测试可传固定值）。

    Returns:
        str: 形如 ``%Y%m%d-%H%M%S-<4 hex>`` 的 id。
    """
    stamp = (datetime.now() if now is None else now).strftime("%Y%m%d-%H%M%S")
    suffix = "".join(secrets.choice(_HEX) for _ in range(4))
    return f"{stamp}-{suffix}"


def prune_runs(root: str | Path | None = None, keep: int = KEEP_RUNS,
               exclude=None, protect=None) -> list[Path]:
    """删掉最旧的运行目录，使 ``runs/`` 下不超过 ``keep`` 个。

    排序键是 ``(目录 mtime, 目录名)``：同一秒内创建的目录按名字（即时间戳 +
    随机后缀）决出先后，排序稳定。

    Args:
        root: 产物根目录（``None`` 走 :func:`runs_root`）。
        keep: 保留个数。
        exclude: 不许删的目录（绝对路径或名字）；调度器会把**还没结束的任务**
            的目录传进来，否则长任务算完写盘时目录已被别人清掉。
        protect: 同样不删，但**仍计入** keep 的分子——``make_run_dir`` 用它保证
            "刚建的那个"不会被自己触发的淘汰删掉（目录 mtime 相同时按名字排序
            会碰运气，随机后缀可能把新目录排成最旧）。

    Returns:
        list[Path]: 被删掉的目录（从最旧到次新）；目录不存e时返回空列表。
    """
    base = runs_root(root)
    if not base.is_dir():
        return []
    keep_paths = {str(Path(item).resolve()) for item in (exclude or ())}
    protected = {str(Path(item).resolve()) for item in (protect or ())}
    directories = [path for path in base.iterdir() if path.is_dir()]
    candidates = [path for path in directories
                  if str(path.resolve()) not in keep_paths
                  and str(path.resolve()) not in protected]
    # keep 的分子算**全部**目录（含受保护的），否则新目录会让数量永远超一格
    excess = len(directories) - len(keep_paths) - max(0, int(keep))
    if excess <= 0 or not candidates:
        return []
    # mtime 相同时用 st_mtime_ns 再比一次；仍相同才轮到名字（随机后缀，不保证顺序）
    candidates.sort(key=lambda path: (path.stat().st_mtime_ns, path.name))
    removed: list[Path] = []
    for path in candidates[:excess]:
        shutil.rmtree(path, ignore_errors=True)
        removed.append(path)
    return removed


def make_run_dir(root: str | Path | None = None, run_id: str | None = None,
                 keep: int = KEEP_RUNS, exclude=None) -> tuple[str, Path]:
    """新建一个运行目录并执行保留策略。

    Args:
        root: 产物根目录（``None`` 走 :func:`runs_root`）。
        run_id: 指定 id；``None`` 则用 :func:`new_run_id` 生成（重名时重试）。
        keep: ``runs/`` 下保留的目录个数。
        exclude: 不参与淘汰的目录（活跃任务的目录）。

    Returns:
        ``(run_id, run_dir)``：目录已创建（产物根目录会一并创建）。
    """
    base = runs_root(root)
    base.mkdir(parents=True, exist_ok=True)
    keep = max(1, int(keep))          # 至少留下刚建的这个
    candidate = run_id
    while True:
        if candidate is None:
            candidate = new_run_id()
        run_dir = base / candidate
        try:
            run_dir.mkdir(parents=False, exist_ok=False)
        except FileExistsError:
            if run_id is not None:
                raise          # 调用方指定了 id，冲突就是调用方的错
            candidate = None   # 随机后缀撞车：换一个再试
            continue
        break
    prune_runs(base, keep, exclude, protect=(run_dir,))
    return candidate, run_dir


@dataclass
class Job:
    """一条后台任务的共享状态（工作线程写，UI 线程读）。

    字段与设计文档 §7 一致；``progress`` 里固定四个键
    ``stage`` / ``index`` / ``total`` / ``fraction``，供进度条直接绑定。
    """

    id: str
    kind: str
    state: str
    progress: dict
    log: list[str]
    result: dict | None
    error: str | None
    run_dir: Path
    cancel: threading.Event
    #: 累计写入过的日志行数（单调递增，含已被环形缓冲挤掉的）。
    _log_total: int = 0
    #: 被环形缓冲挤掉的日志行数（``log[0]`` 的绝对序号）。
    _dropped: int = 0
    _lock: threading.Lock = field(default_factory=threading.Lock, repr=False)
    #: 变更订阅者（见 :meth:`subscribe`）。**只在锁内增删、锁外调用**。
    _listeners: list = field(default_factory=list, repr=False)

    def append_log(self, line: str) -> None:
        """往环形缓冲追加一行（线程安全，超过 :data:`LOG_LIMIT` 丢最旧的）。"""
        text = str(line).rstrip("\n")
        with self._lock:
            self._log_total += 1
            self.log.append(text)
            if len(self.log) > LOG_LIMIT:
                excess = len(self.log) - LOG_LIMIT
                del self.log[:excess]
                self._dropped += excess
        self._notify()

    def set_progress(self, stage: str, index: int = 0, total: int = 0) -> None:
        """更新进度（线程安全）。``fraction`` 由 ``index/total`` 现算。

        Args:
            stage: 阶段名（中文短语，如 ``解算 IK``）。
            index: 当前进度（已完成数）。
            total: 该阶段总数；``<= 0`` 时 ``fraction`` 记 0。
        """
        index = int(index)
        total = int(total)
        fraction = 0.0 if total <= 0 else max(0.0, min(1.0, index / total))
        with self._lock:
            self.progress = {"stage": str(stage), "index": index,
                             "total": total, "fraction": fraction}
        self._notify()

    def snapshot(self, log_from: int = 0) -> dict:
        """当前状态快照（给前端轮询）。

        Args:
            log_from: 已消费到的**绝对行号**（0 表示从头要）。行号按累计写入
                计数，因此环形缓冲挤掉旧行后仍能正确续传；请求早于缓冲起点时
                自动抬到起点。

        Returns:
            dict: ``{"id", "kind", "state", "progress", "log", "log_from",
            "log_total", "dropped", "result", "error", "run_dir"}``；
            ``log`` 是 ``log_from`` 之后的新行，``log_from`` 是下次调用要传的
            游标（= ``log_from + len(log)``）。
        """
        with self._lock:
            start = max(int(log_from), self._dropped)
            lines = list(self.log[start - self._dropped:])
            return {"id": self.id,
                    "kind": self.kind,
                    "state": self.state,
                    "progress": dict(self.progress),
                    "log": lines,
                    "log_from": start + len(lines),
                    "log_total": self._log_total,
                    "dropped": self._dropped,
                    "result": self.result,
                    "error": self.error,
                    "run_dir": str(self.run_dir)}

    def set_state(self, state: str, *, result: dict | None = None,
                  error: str | None = None) -> None:
        """改状态（线程安全）；``done`` 时才保留 ``result``。"""
        with self._lock:
            self.state = state
            self.result = result if state == "done" else None
            self.error = error
        self._notify()

    # ------------------------------------------------------------------ 订阅
    def subscribe(self, callback: Callable[[Job], Any]) -> None:
        """注册变更回调（**在工作线程里被调用**，桌面版把轮询换成信号用）。

        ``append_log`` / ``set_progress`` / ``set_state`` 每改完一次就在**锁外**
        调一次 ``callback(self)``；也就是说回调可能来自工作线程、也可能来自
        调用方线程（例如 UI 线程自己 ``set_progress``）。因此回调里：

        * 做 UI 的事情只能 ``signal.emit(...)``（Qt 会排队到对象所属线程）；
        * 读状态请走 :meth:`snapshot`（同一把锁，锁外可重入，不会死锁）。

        ``snapshot()`` 与网页版的轮询行为完全不变，两条路互不影响。

        Args:
            callback: ``callback(job)``；同一对象重复注册只生效一次。
        """
        with self._lock:
            if any(existing == callback for existing in self._listeners):
                return
            self._listeners.append(callback)

    def unsubscribe(self, callback: Callable[[Job], Any]) -> None:
        """注销订阅（没注册过就静默忽略，便于在 ``finally`` 里无条件调用）。"""
        with self._lock:
            for index, existing in enumerate(self._listeners):
                if existing is callback or existing == callback:
                    del self._listeners[index]
                    return

    def _notify(self) -> None:
        """在**锁外**依次回调订阅者（监听器自己的异常只记录，不拖垮任务）。

        先复制一份订阅者列表再调用：回调里增删订阅（或干脆 ``unsubscribe``
        自己）都不会破坏本次遍历；也不持锁调用，避免与 ``snapshot`` 互锁。
        """
        with self._lock:
            if not self._listeners:      # 网页版（不订阅）就多一次空判断，没别的开销
                return
            listeners = list(self._listeners)
        for callback in listeners:
            try:
                callback(self)
            except Exception:       # noqa: BLE001 - 监听器的 bug 不该弄挂解算
                # 吞掉但不能静默：监听器（桌面端的 Qt 信号转发）出错时解算照跑，
                # 只会“界面不再更新”，没有日志根本无从排查。
                _LOGGER.warning("订阅回调异常（任务 %s，回调 %r）",
                                self.id, callback, exc_info=True)
                continue


class _JobStream(io.TextIOBase):
    """把 ``print`` 的字节流按行送进 :class:`Job` 的环形缓冲。

    ``write`` 可能从**多条线程**并发进入：``redirect_stdout`` 换的是进程级的
    ``sys.stdout``，任务线程自己会 ``print``，别的线程（例如 UI 线程上的
    ``core.preview_domain``/``convert_urdf``）也可能在同一时刻写；而
    ``self._buffer += text`` 是“读属性-拼接-写属性”三步，中间被切线程就会
    丢字。所以行缓冲自己带锁（往 :class:`Job` 里追加本来就有锁，不冲突）。

    **已知限制**：``redirect_stdout`` 是进程级的，不是线程局部的——任务运行
    期间**其它线程**的 ``print`` 也会被收进这个任务的日志（而不是服务端终端），
    反过来 UI 线程临时 ``redirect_stdout`` 时任务的那几行也可能进不了日志。
    单进程/单任务的设计下这个副作用可以接受（比给所有算法函数改成传 logger
    简单得多），但排查服务端问题时要知道日志里混着 UI 线程的输出。
    """

    def __init__(self, job: Job) -> None:
        super().__init__()
        self._job = job
        self._buffer = ""
        self._lock = threading.Lock()

    def writable(self) -> bool:      # noqa: D102 - TextIOBase 协议
        return True

    def write(self, text: str) -> int:      # noqa: D102 - TextIOBase 协议
        with self._lock:
            self._buffer += text
            lines = []
            while "\n" in self._buffer:
                line, self._buffer = self._buffer.split("\n", 1)
                lines.append(line)
        for line in lines:               # 追加到 Job 自己会加锁，不放在上面那层
            self._job.append_log(line)
        return len(text)

    def flush(self) -> None:         # noqa: D102 - TextIOBase 协议
        with self._lock:
            pending, self._buffer = self._buffer, ""
        if pending:
            self._job.append_log(pending)


class JobRegistry:
    """任务登记表：单并发队列 + 线程安全快照。

    一条 daemon 工作线程消费队列，因此同一时间只有一个任务在算；``submit``
    永不阻塞（只入队），异常一律在任务内部转成 ``failed``，不会冒到 UI 线程。
    """

    def __init__(self, root: str | Path | None = None,
                 keep: int = KEEP_RUNS,
                 max_finished: int = MAX_FINISHED) -> None:
        """登记表。

        Args:
            root: 产物根目录（``None`` 走 :func:`runs_root`）。
            keep: ``runs/`` 下保留的目录个数。
            max_finished: 最多保留多少条已结束的任务（未结束的永不淘汰）。
        """
        self.root = runs_root(root)
        self.keep = int(keep)
        self.max_finished = max(0, int(max_finished))
        self._jobs: dict[str, Job] = {}
        self._lock = threading.Lock()
        self._queue: queue.Queue = queue.Queue()
        self._worker: threading.Thread | None = None

    def active_dirs(self) -> set[str]:
        """还没结束的任务的目录（清理时必须跳过，否则会删掉正在写的产物）。"""
        with self._lock:
            return {str(job.run_dir.resolve()) for job in self._jobs.values()
                    if job.state not in TERMINAL_STATES}

    def _prune_finished(self) -> int:
        """只留最近 :attr:`max_finished` 条**已结束**的任务，返回丢掉几条。

        插入顺序就是提交顺序（``dict`` 保序），所以“最旧的”= 最早插入的。
        未结束（``pending`` / ``running``）的任务**永远不会**被选中：它们还
        要写产物、还要被轮询。
        """
        with self._lock:
            finished = [job_id for job_id, job in self._jobs.items()
                        if job.state in TERMINAL_STATES]
            excess = len(finished) - self.max_finished
            for job_id in finished[:max(0, excess)]:
                del self._jobs[job_id]
            return max(0, excess)

    # ------------------------------------------------------------------ 提交
    def submit(self, kind: str, fn: Callable[[Job], Any]) -> str:
        """登记一条任务并返回 id（不阻塞、不执行）。

        Args:
            kind: 任务种类，见 :data:`KINDS`（仅作标记，不参与调度）。
            fn: ``fn(job)``，在工作线程里执行；返回值放进 ``job.result``。

        Returns:
            str: 任务 id（同时也是 ``runs/<id>/`` 的目录名）。
        """
        # 先淘汰最旧的已结束任务：新任务要登记，正在跑的那个不能占份额
        self._prune_finished()
        run_id, run_dir = make_run_dir(self.root, keep=self.keep,
                                       exclude=self.active_dirs())
        job = Job(id=run_id, kind=str(kind), state="pending",
                  progress={"stage": "排队", "index": 0, "total": 0,
                            "fraction": 0.0},
                  log=[], result=None, error=None, run_dir=run_dir,
                  cancel=threading.Event())
        with self._lock:
            self._jobs[run_id] = job
        self._ensure_worker()
        self._queue.put((run_id, fn))
        return run_id

    def clean(self, keep: int | None = None) -> list[Path]:
        """按保留策略清理，跳过还没结束的任务目录。"""
        return prune_runs(self.root, self.keep if keep is None else int(keep),
                          exclude=self.active_dirs())

    # ------------------------------------------------------------------ 查询
    def get(self, job_id: str) -> Job | None:
        """按 id 取任务（不存在返回 ``None``）。"""
        with self._lock:
            return self._jobs.get(job_id)

    def snapshot(self, job_id: str, log_from: int = 0) -> dict | None:
        """按 id 取快照（不存在返回 ``None``）；见 :meth:`Job.snapshot`。"""
        job = self.get(job_id)
        return None if job is None else job.snapshot(log_from)

    def jobs(self) -> list[Job]:
        """全部任务（按提交顺序）。"""
        with self._lock:
            return list(self._jobs.values())

    # ------------------------------------------------------------------ 取消
    def request_cancel(self, job_id: str) -> bool:
        """请求取消。

        只是置标志：正在跑的任务在下一个阶段边界自行收尾（IK 不会被打断），
        还没开始的任务则直接以 ``cancelled`` 结束。

        Args:
            job_id: 任务 id。

        Returns:
            bool: 是否受理（任务不存在或已结束返回 ``False``）。
        """
        with self._lock:
            job = self._jobs.get(job_id)
            # 读状态与置标志必须在同一把锁里：否则 worker 可能刚把 state 写成
            # done，这边仍看到 running 并返回 True，用户以为取消了但结果照旧落盘。
            if job is None or job.state in TERMINAL_STATES:
                return False
            job.cancel.set()
            return True

    # -------------------------------------------------------------- 工作线程
    def shutdown(self, timeout: float = 5.0) -> None:
        """停掉工作线程（测试收尾用；会等当前任务结束）。"""
        with self._lock:
            worker = self._worker
        if worker is None:
            return
        self._queue.put(None)
        worker.join(timeout)

    def _ensure_worker(self) -> None:
        """懒启动唯一的工作线程。"""
        with self._lock:
            if self._worker is not None and self._worker.is_alive():
                return
            self._worker = threading.Thread(target=self._loop,
                                            name="laser-web-jobs", daemon=True)
            self._worker.start()

    def _loop(self) -> None:
        """工作线程主循环：串行消费队列。"""
        while True:
            item = self._queue.get()
            try:
                if item is None:
                    return
                job_id, fn = item
                job = self.get(job_id)
                if job is not None:
                    self._run_one(job, fn)
            finally:
                self._queue.task_done()

    def _run_one(self, job: Job, fn: Callable[[Job], Any]) -> None:
        """跑一个任务：捕获 stdout、把任何异常转成 failed，最后收尾。

        注意 ``redirect_stdout`` 是**进程级**的：任务执行期间服务端自己的
        ``print`` 也会落进这条任务的日志（详见 :class:`_JobStream` 的已知限制）。
        本工具单进程单任务，这个副作用比「给每个函数传一个 logger」简单得多，
        也正好让界面日志等价于终端输出。
        """
        if job.cancel.is_set():
            job.append_log("已取消（任务未开始）")
            job.set_state("cancelled")
            self._discard_partial_run(job)
            return
        job.set_state("running")
        job.set_progress("启动", 0, 1)
        stream = _JobStream(job)
        try:
            with contextlib.redirect_stdout(stream):
                result = fn(job)
            stream.flush()
        except BaseException as error:          # noqa: BLE001 - strict 模式会 sys.exit
            stream.flush()
            cancelled = job.cancel.is_set() or isinstance(error, JobCancelled)
            if cancelled:
                job.append_log("已取消")
                job.set_state("cancelled")
                self._discard_partial_run(job)
            else:
                message = f"{type(error).__name__}: {error}"
                job.append_log(f"[失败] {message}")
                job.set_state("failed", error=message)
            return
        if job.cancel.is_set():
            job.append_log("已取消（任务已结束，结果丢弃）")
            job.set_state("cancelled")
            self._discard_partial_run(job)
        else:
            job.set_state("done", result=result)
            job.set_progress("完成", 1, 1)

    @staticmethod
    def _discard_partial_run(job: Job) -> None:
        """取消后清掉「半成品」运行目录；已经落下结果 JSON 的则保留。

        取消只在阶段边界生效，所以最常出现的是「出图之后、写文件之前」：目录
        里只有一个 ``plot.png``。这种目录既不能导出 GP（没 ``scan.json``），
        又会在工具页的运行列表里伪装成一次正常运行，不如删掉。

        反过来，若取消发生在产物都写完、结果被丢弃那一刻，目录是**完整**的
        （``scan.json`` / ``iso.json`` 已在），保留才能事后回看——所以判据是
        「有没有结果 JSON」，而不是「是不是取消的」。

        删不掉（竞态/权限）也不报错：取消本来就该安静结束。
        """
        run_dir = Path(job.run_dir)
        try:
            for name in RESULT_ARTIFACTS:
                if (run_dir / name).is_file():
                    job.append_log(f"保留完整运行目录 {run_dir}")
                    return
            if not run_dir.is_dir():
                return
            shutil.rmtree(run_dir, ignore_errors=True)
        except OSError:                 # 竞态下手慢：别人已经删了 / 没权限
            return
        job.append_log(f"已删除半成品运行目录 {run_dir}")


_default_registry: JobRegistry | None = None
_default_lock = threading.Lock()


def get_registry(root: str | Path | None = None) -> JobRegistry:
    """进程级共享登记表（生成页用；NiceGUI 单进程无需多实例）。

    Args:
        root: 首次调用时生效的产物根目录；``None`` 走 :func:`runs_root`。

    Returns:
        JobRegistry: 共享实例。
    """
    global _default_registry
    with _default_lock:
        if _default_registry is None:
            _default_registry = JobRegistry(root)
        return _default_registry
