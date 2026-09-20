"""Laptop side of PROTOCOL.md s8: watch the telemetry link and the board's own `armed` flag. Pure logic, no sockets.

  from laptop.control.link import TelemWatchdog
  wd = TelemWatchdog()                      # gains from config.FOLLOW: TELEM_LOST_MS, BOARD_OFF_N, AGE_WARN_MS
  wd.arm(now)                               # when the operator arms
  for m, _ in tel_in.recv_all(only_from=esp): warn = wd.telem(m, now, armed)   # EVERY frame, not recv_latest
  reason = wd.check(armed, now)             # once per loop tick; disarm when it returns a string

Pass `now` from any monotonic clock: time.monotonic() in follow_me / pilot, sim seconds in archive/tools/scenarios.py.
Why every frame: after a WiFi hiccup > 500 ms the board trips its failsafe and reports armed:0 / age >= 500 in ONE
frame, then re-arms on the next command 50 ms later. recv_latest would drop that frame. Why N > 3 for the consecutive
armed:0 rule: right after arming the board reports armed:0 for 1-3 frames until the first arm:1 reaches it.
"""
from .. import config
from .protocol import FAILSAFE_MS

G = config.FOLLOW


class TelemWatchdog:
    def __init__(self, lost_ms=None, board_off_n=None, age_warn_ms=None):
        self.lost_s = (G["TELEM_LOST_MS"] if lost_ms is None else lost_ms) / 1000.0
        self.board_off_n = G["BOARD_OFF_N"] if board_off_n is None else board_off_n
        self.age_warn_ms = G["AGE_WARN_MS"] if age_warn_ms is None else age_warn_ms
        self.t_last = None            # loop time of the last telemetry frame
        self.n_off = 0                # consecutive frames saying armed:0 while we are armed
        self.tripped = None           # reason string once the board reported a failsafe trip
        self.age = -1                 # the board's 'ms since last command' from the last frame
        self.board_armed = None
        self._t_warn = -1e9

    def arm(self, now):
        """Operator armed: silence is counted from now (a board that never talked gets lost_ms to show up)."""
        self.n_off, self.tripped = 0, None
        if self.t_last is None:
            self.t_last = now

    def telem(self, m, now, laptop_armed):
        """Feed one telemetry frame. Returns a warning string to print (rate-limited to one per 5 s), or None."""
        self.t_last = now
        self.board_armed = int(m.get("armed", 0)) == 1
        self.age = int(m.get("age", -1))
        if laptop_armed and not self.board_armed:
            self.n_off += 1
            if self.age >= FAILSAFE_MS:
                self.tripped = f"board failsafe tripped (it had no command for {self.age} ms)"
        else:
            self.n_off = 0
        if self.age > self.age_warn_ms and now - self._t_warn > 5.0:
            self._t_warn = now
            return f"link slow: the board last heard from us {self.age} ms ago (failsafe at {FAILSAFE_MS})"
        return None

    def check(self, laptop_armed, now):
        """Once per loop tick. Returns the reason to disarm, or None."""
        if not laptop_armed:
            self.n_off, self.tripped = 0, None
            return None
        if self.t_last is None:
            return "no telemetry from the board"
        if now - self.t_last > self.lost_s:
            return f"telemetry silent for {now - self.t_last:.1f} s"
        if self.tripped:
            return self.tripped
        if self.n_off >= self.board_off_n:
            return f"board reports motors off in {self.n_off} frames (age {self.age} ms)"
        return None
