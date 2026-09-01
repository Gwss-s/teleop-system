"""AsyncChunkSource: non-blocking base-policy chunk consumption (sim-clock).

Solves the 50%-duty-cycle stutter of synchronous requery: with a ~510 ms
inference round-trip and exec_horizon=10 (=500 ms at 20 Hz), a blocking client
freezes every half second. Here a background thread overlaps inference with
execution:

  * sim-clock indexing: a chunk requested with the obs of sim-step t0 provides
    actions for steps t0..t0+L-1; at step t we serve chunk[t - t0], so a chunk
    that arrives late is consumed from the correct offset, not from 0;
  * prefetch: one outstanding request, re-fired at every chunk swap -> the
    ~0.5 s inference rides under the ~0.5 s execution horizon;
  * takeover-aware: during human takeover the same machinery keeps serving the
    counterfactual a_base (shadow query) without ever blocking the human loop;
  * rebase(): after handback, drop stale plans and re-plan from the real state;
    next_a_base() returns None until the fresh chunk lands (caller holds pose);
  * next_a_base(sync=True) -- OFFICIAL-EVAL consumption for AUTO execution:
    each chunk must be conditioned on the obs of its first executed step and
    only its first exec_horizon actions run (age 0..h-1, exactly eval_client's
    blocking requery). While the fresh chunk is in flight it returns None and
    the caller freezes the sim (sim-time invisible). Results whose origin
    doesn't match the current step (handback races) are dropped, never served.
    Rationale: over a tunnel the prefetch round-trip exceeds the horizon, so
    async consumption silently degrades to 20-45-step open loop -- a different
    (much worse) policy than the one evaluated.

Only start_episode() and sync-mode segment boundaries block.
"""
import threading

import numpy as np


class AsyncChunkSource:
    def __init__(self, conn, send_msg, recv_msg, exec_horizon=10, chunk_len=50):
        self.conn = conn
        self._send, self._recv = send_msg, recv_msg
        self.h, self.L = int(exec_horizon), int(chunk_len)
        self.cv = threading.Condition()
        self._req = None          # (request_dict, origin_step) waiting to be sent
        self._result = None       # (chunk[L,act], origin_step) ready to swap in
        self.cur, self.origin = None, 0
        self._need_prefetch = False
        self._busy = False        # worker 正在 send/recv 一个请求(在途)
        self._alive = True
        self._err = None      # worker 致命错误(主循环 _check_err 抛出)
        self.starved = 0          # diagnostic counter
        threading.Thread(target=self._worker, daemon=True).start()

    # -- background inference thread ----------------------------------------
    def _worker(self):
        while self._alive:
            with self.cv:
                while self._req is None and self._alive:
                    self.cv.wait(0.1)
                if not self._alive:
                    return
                req, origin = self._req
                self._req = None
                self._busy = True
            try:
                self._send(self.conn, req)
                chunk = np.asarray(self._recv(self.conn), dtype=np.float32)
            except Exception as e:  # 网络/解包错误: 存下并唤醒主循环大声崩,不许无声挂死
                with self.cv:
                    self._err = e
                    self._alive = False
                    self.cv.notify_all()
                return
            with self.cv:
                self._result = (chunk, origin)
                self._busy = False
                self.cv.notify_all()

    def _check_err(self):
        if self._err is not None:
            raise RuntimeError(f"chunk source worker died: {self._err!r}") from self._err

    def _fire(self, req, step):
        with self.cv:
            self._req = (req, step)
            self.cv.notify_all()

    # -- main-loop API --------------------------------------------------------
    def start_episode(self, req, step=0):
        """Cold start: blocking single inference, then the pipeline stays full."""
        self._check_err()
        with self.cv:
            self._result = None
        self.cur, self.origin = None, 0
        self._fire(req, step)
        with self.cv:
            while self._result is None:
                if self._err is not None:
                    break
                self.cv.wait(0.1)
            self._check_err()
            self.cur, self.origin = self._result
            self._result = None
        self._need_prefetch = True

    def next_a_base(self, req, step, sync=False):
        """sync=False(接管期影子查询): 非阻塞,sim-clock 对位消费,晚到的 chunk
        从正确偏移接着用;返回 None 仅在 rebase 在途(caller 保持位姿)。
        sync=True(AUTO 执行,官方评测协议): chunk 以"段首当帧 obs"为条件,只执行
        前 exec_horizon 步(age 恒 0..h-1);段耗尽时用当前 obs 发请求并返回 None,
        调用方暂停世界(不步进)直到新鲜 chunk 落地 —— 与 eval_client 的阻塞重
        规划逐步等价。origin 与当前步不符的回包(换手/开局竞态遗留)直接丢弃。"""
        self._check_err()
        if sync:
            with self.cv:
                if self.cur is not None:
                    idx = step - self.origin
                    if 0 <= idx < self.h:
                        return self.cur[idx]          # 段内:执行本段前 h 步
                if self._result is not None:
                    chunk, origin = self._result
                    self._result = None
                    if origin == step:                # 新鲜落地:新段从 idx 0 开始
                        self.cur, self.origin = chunk, origin
                        return self.cur[0]
                    # origin != step: 陈旧回包,丢弃继续等
                if self._req is None and not self._busy:
                    self._fire(req, step)             # Condition 的锁可重入,安全
            return None                               # 世界暂停,等新鲜 chunk
        with self.cv:
            if self._result is not None:
                chunk, origin = self._result
                if self.cur is None or step - self.origin >= self.h:
                    self.cur, self.origin = chunk, origin
                    self._result = None
                    self._need_prefetch = True
        if self.cur is None:
            return None                      # rebase in flight
        if self._need_prefetch:
            self._need_prefetch = False
            self._fire(req, step)            # overlap inference with execution
        idx = step - self.origin
        if idx >= self.L:                    # starvation safety: repeat last action
            self.starved += 1
            idx = self.L - 1
        return self.cur[max(idx, 0)]

    def rebase(self, req, step):
        """After human handback: drop stale plans, re-plan from the real state.
        next_a_base() returns None (~1 inference) until the fresh chunk lands."""
        with self.cv:
            self._result = None
        self.cur = None
        self._need_prefetch = False
        self._fire(req, step)

    def close(self):
        self._alive = False
        with self.cv:
            self.cv.notify_all()
