# -*- coding: utf-8 -*-
"""任务桥：把 ``laser_web.jobs`` 的轮询换成 Qt 信号（S3）。

线程模型（对应设计文档 §3.2 / §3.3）
------------------------------------
``jobs.JobRegistry`` 里只有**一条工作线程**，``core.run_scan`` 在那条线程里跑
（IK 吃满一个核）。本模块只做一件事：把那边的"状态变了"翻译成 GUI 线程上的
三个信号。

1. 工作线程每改一次状态（``append_log`` / ``set_progress`` / ``set_state``）就
   调一次 :meth:`Job.subscribe` 的监听器，也就是本类的 :meth:`_on_job_change`；
2. 监听器**只做一件事**：``self._notified.emit(job)``。Qt 的 auto connection
   发现"emit 的线程 ≠ 接收对象的线程"会自动排队（queued connection），于是槽
   :meth:`_on_notified` 一定在 **GUI 线程**执行。监听器里**绝不**碰
   QWidget / QTimer / QMessageBox（那是最典型的跨线程崩溃来源），也绝不
   ``blockingQueuedConnection``（工作线程会一直等 GUI 线程，卡住解算）；
3. GUI 线程收到通知**不立即 emit 给页面**，而是启动一个 :data:`FLUSH_MS` 的
   单次 ``QTimer``：同一批里的几十次变化合成一次 :meth:`_flush`——用
   ``job.snapshot(cursor)`` 取**增量日志**逐行 emit、进度 emit 一次、终态再
   emit 一次 ``finished``。日志游标与网页版的 ``snapshot(log_from)`` 是同一套
   语义（绝对行号，环形缓冲挤掉旧行也不会重复/漏行）；
4. **终态立即 flush**（不等定时器）：取消要"点了就能看见"，最后一行日志与
   ``finished`` 不能迟到 180 ms 以上。

为什么不直接在工作线程里启动 ``QTimer``：Qt 的定时器只能在自己的线程里
start/stop，跨线程操作是未定义行为。所以定时器只存在于 GUI 线程，工作线程
只发信号。

``JobBridge`` 不知道任何页面细节：谁连信号、怎么画，都是页面的事。
"""

from __future__ import annotations

from typing import Any

from laser_web import jobs

from . import paths
from .qt_compat import QtCore, Signal

#: 合并刷新间隔（毫秒）。150~200 之间：太快没有合并效果，太慢日志看着像卡住。
FLUSH_MS = 180

#: ``jobs`` 的日志环形缓冲上限（行）。页面上的说明文字引用它，不手抄数字：
#: 源缓冲比日志控件（``LOG_WIDGET_LIMIT``）小，超出就真的没了。
LOG_LIMIT = jobs.LOG_LIMIT

#: 任务状态 -> 界面文案（页面直接用，避免各处各写一份）。
STATE_TEXT = {"pending": "排队中", "running": "进行中", "done": "完成",
              "failed": "失败", "cancelled": "已取消"}


def adopt_running_job(registry, kind: str):
    """找一个还在跑的同类型任务，供**共用同一个登记表**的页面重建时接管。

    语义照抄网页版的 ``laser_web/pages/__init__.py::adopt_running_job``（那边
    的模块 import 了 nicegui，桌面版不能直接 import，所以照抄这 10 行）。任务
    跑在 ``jobs`` 的工作线程里，与页面对象的生死无关；用**同一个** ``registry``
    重建页面（换掉这一页的控件 / 桥复用）时若不接管，任务既看不到也取消不了，
    还会把新提交的任务堵在“排队中”。

    **边界（不要说成“崩溃重启后接管”）**：本模块的 :class:`JobBridge` 默认
    **每个桥自己 ``new`` 一个进程内 ``JobRegistry``**，任务/日志/结果都只活在
    本进程内存里。所以：

    * 同进程、同一个登记表 → 能接管；
    * 另开一个窗口/进程（哪怕同一个 ``runs/`` 目录）→ **接管不了**，新桥的
      登记表是空的；
    * 进程退出（正常或崩溃）→ 什么都不剩。跨进程接管要把任务状态落盘，
      S3 不做。

    Args:
        registry: ``jobs.JobRegistry``。
        kind: ``"scan"`` / ``"repair"`` / ``"iso"``。

    Returns:
        ``jobs.Job`` 或 ``None``（没有在跑的同类型任务）。
    """
    for job in registry.jobs():
        if job.kind == str(kind) and job.state not in jobs.TERMINAL_STATES:
            return job
    return None


class JobBridge(QtCore.QObject):
    """一条任务 ↔ 一个页面：进度 / 日志 / 终态三条信号，全部在 GUI 线程发出。

    Signals:
        progress: ``{"stage", "index", "total", "fraction", "state"}``；前四个键
            与 ``jobs.Job.progress`` 一致，``state`` 是多余的方便字段（页面据此
            显示"进行中/完成"）。
        log_line: 增量日志的**一行**（按 ``snapshot(log_from)`` 的游标取，绝不
            重复；环形缓冲挤掉的旧行本来就不存在了，也不会假装有）。
        finished: ``{"state", "result", "error", "run_id", "run_dir"}``；每个任务
            只发一次（重放的重复通知会被吞掉）。

    Attributes:
        registry: ``jobs.JobRegistry``（默认按 :func:`laser_desktop.paths.runs_root`
            新建一个，与网页版共用同一个 ``runs/``）。
        job_id: 当前任务 id（没挂任务时为 ``None``）。
        job: 当前 ``jobs.Job``（没有则为 ``None``；结束后仍保留，便于回看）。
    """

    progress = Signal(object)
    log_line = Signal(str)
    finished = Signal(object)
    #: 内部信号：工作线程 → GUI 线程的"有变化"通知（跨线程 auto=queued）。
    _notified = Signal(object)

    def __init__(self, registry=None, parent=None, flush_ms: int = FLUSH_MS,
                 root=None) -> None:
        """建桥。

        Args:
            registry: 现成的 ``jobs.JobRegistry``（测试用假登记表 / 临时目录）；
                ``None`` 时按 ``root`` 新建一个。
            parent: 父 QObject（一般传页面，跟着页面一起销毁）。
            flush_ms: 合并刷新间隔（毫秒）。
            root: 新建登记表时的 ``runs/`` 根目录；``None`` 走
                :func:`laser_desktop.paths.runs_root`（打包后是用户数据目录）。
        """
        super().__init__(parent)
        if registry is None:
            registry = jobs.JobRegistry(
                root=paths.runs_root() if root is None else root)
        self.registry = registry
        self.job_id: str | None = None
        self.job: Any = None
        self._cursor = 0
        self._dropped = 0
        self._finished_sent = False
        self._timer = QtCore.QTimer(self)
        self._timer.setSingleShot(True)
        self._timer.setInterval(max(1, int(flush_ms)))
        self._timer.timeout.connect(self._flush)
        self._notified.connect(self._on_notified)

    # ------------------------------------------------------------------ 提交
    def submit(self, kind: str, fn) -> str:
        """提交任务并挂上订阅（``fn(job)`` 在工作线程里跑）。

        Args:
            kind: 任务种类（``"scan"`` / ``"repair"`` / ``"iso"``）。
            fn: ``fn(job)``；返回值进 ``job.result``。

        Returns:
            str: 任务 id（= ``runs/<id>/`` 的目录名）。
        """
        job_id = self.registry.submit(kind, fn)
        self.attach(job_id)
        return job_id

    def attach(self, job_id: str):
        """接管一条任务（本页新提交的，或重建页面时捡回来的在跑任务）。

        接手时游标归零并立刻读一次现状：即使任务已经跑了一半（甚至已经结束），
        页面也能马上看到当前进度、缓冲里的日志与终态。

        Args:
            job_id: 任务 id。

        Returns:
            ``jobs.Job`` 或 ``None``（id 不存在）。
        """
        job = self.registry.get(job_id)
        if job is None:
            return None
        if job is self.job:
            return job
        self.detach()
        self.job = job
        self.job_id = job.id
        self._cursor = 0
        self._dropped = 0
        self._finished_sent = False
        job.subscribe(self._on_job_change)
        self._flush()           # 补一次现状（可能已完成：finished 立刻就到）
        return job

    def detach(self) -> None:
        """放开当前任务（换任务 / 页面关闭时调；不会取消任务本身）。"""
        job, self.job, self.job_id = self.job, None, None
        self._timer.stop()
        if job is not None:
            job.unsubscribe(self._on_job_change)

    def cancel(self) -> bool:
        """请求取消当前任务（阶段边界生效，与网页版同语义）。

        Returns:
            bool: 是否受理（没挂任务 / 已结束返回 ``False``）。
        """
        if self.job_id is None:
            return False
        return self.registry.request_cancel(self.job_id)

    @property
    def is_running(self) -> bool:
        """当前任务是否还没结束（``pending`` / ``running``）。"""
        return self.job is not None and self.job.state not in jobs.TERMINAL_STATES

    # -------------------------------------------------- 工作线程 → GUI 线程
    def _on_job_change(self, job) -> None:
        """订阅回调：**跑在工作线程里**，只允许 emit 信号。

        Args:
            job: 变化了的任务（本桥只关心自己那条）。
        """
        try:
            self._notified.emit(job)
        except RuntimeError:        # QObject 已销毁（窗口关得比任务快）
            pass

    def _on_notified(self, job) -> None:
        """GUI 线程：收到"有变化"，决定是立刻刷新还是攒到定时器。

        Args:
            job: 变化了的任务。
        """
        if job is not self.job:     # 旧任务的迟到通知（已经 detach 过）
            return
        if job.state in jobs.TERMINAL_STATES:
            self._flush()           # 终态不排队：取消/失败要立刻看见
        elif not self._timer.isActive():
            self._timer.start()     # 已经在跑就不重置：保证最多每 FLUSH_MS 刷一次

    def _flush(self) -> None:
        """把两次刷新之间的变化一次性推给页面（GUI 线程）。

        日志按游标取增量，进度只发最新一份，终态只发一次 ``finished``。
        """
        job = self.job
        if job is None:
            return
        self._timer.stop()          # 手动 flush（终态）时顺手撤掉还没响的那次定时器
        snapshot = job.snapshot(self._cursor)
        self._cursor = snapshot["log_from"]
        # 环形缓冲挤掉的旧行本来就不存在了，但不能不吭声就少一段日志（600 行
        # 只留 200 行时，看上去就像解算中途卡住了）。
        dropped = int(snapshot.get("dropped") or 0)
        if dropped > self._dropped:
            self.log_line.emit(
                f"…已省略 {dropped - self._dropped} 行（源缓冲只留最近 "
                f"{LOG_LIMIT} 行）")
            self._dropped = dropped
        for line in snapshot["log"]:
            self.log_line.emit(str(line))
        progress = dict(snapshot["progress"])
        progress["state"] = snapshot["state"]
        self.progress.emit(progress)
        if snapshot["state"] in jobs.TERMINAL_STATES and not self._finished_sent:
            self._finished_sent = True
            self.finished.emit({
                "state": snapshot["state"],
                "result": snapshot["result"],
                "error": snapshot["error"],
                "run_id": self.job_id,
                "run_dir": snapshot["run_dir"],
            })

    def shutdown(self) -> None:
        """收尾：停掉定时器并注销订阅（页面关闭 / 应用退出时调）。"""
        self.detach()
