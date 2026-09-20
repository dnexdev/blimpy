"""Where positions come from: ONE interface, several implementations, a registry keyed by a spec string.

  from laptop.positioning.sources import make_source
  src = make_source("udp")                                   # live: whatever publishes PROTOCOL s4 on 5007
  src = make_source("udp:5008")                              # another port (e.g. a second vision process)
  src = make_source("replay:data/positioning/<session>", speed=2.0, loop=True)
  src = make_source("sim", world=World(...))                 # in-process simulator (archive/tools/scenarios.py style)
  with src:
      for msg in src.poll():                                 # every PROTOCOL s4 message since the last poll, oldest first
          ...

poll() never blocks and never sleeps: callers keep their own loop rate (15 Hz in follow_me / pilot).
Messages are PROTOCOL.md s4 dicts exactly as produced (vision / sim / replay); laptop/positioning/fuse.py merges
several sources into one stream.

Adding a sensor that is not decided yet (UWB anchors, ToF ranging, phone ARKit, a mocap system...): subclass
PositioningSource, produce s4 dicts in poll() with src="<name>", register the class in REGISTRY. See PlaceholderSource.
"""
import abc, time
from ..control.protocol import STATE_PORT, UdpJson


class PositioningSource(abc.ABC):
    name = "?"                                   # goes into fused messages' srcs={} and into logs

    def start(self):
        """Open sockets / files. Returns self so `with make_source(...) as s:` works."""
        return self

    @abc.abstractmethod
    def poll(self):
        """-> list of PROTOCOL s4 dicts received since the last call, oldest first. Never blocks."""

    def stop(self):
        pass

    def __enter__(self):
        return self.start()

    def __exit__(self, *exc):
        self.stop()


class UdpStateSource(PositioningSource):
    """PROTOCOL s4 datagrams on a UDP port (5007 = localize.py / mono.py / fake_esp32 --sim).
    CAVEAT: UdpJson binds with SO_REUSEADDR, and unicast datagrams go to ONE bound socket, so a second listener
    on 5007 steals from follow_me / pilot. Only use on 5007 when no controller runs (record.py, capture_place.py)."""
    name = "udp"

    def __init__(self, port=STATE_PORT, only_from=None):
        self.port, self.only_from, self.sock = int(port), only_from, None

    def start(self):
        if self.sock is None:
            self.sock = UdpJson(self.port)
        return self

    def poll(self):
        if self.sock is None:
            self.start()
        return [obj for obj, _ in self.sock.recv_all(self.only_from)]

    def stop(self):
        if self.sock is not None:
            self.sock.sock.close(); self.sock = None


class ReplaySource(PositioningSource):
    """Rows of a recorded session (laptop/positioning/session.py), paced on the RECORDER's row clock (t_ms).
    data.t is kept as recorded (consumers only use differences), shifted by a constant per lap when looping so it
    never runs backwards. speed <= 0 = no pacing: everything remaining comes out on the first poll (tests, batch).
    kinds: which row kinds to replay; poll() returns state messages only, poll_kinds() returns (kind, data) pairs."""
    name = "replay"
    LAP_GAP_MS = 1000

    def __init__(self, session_dir, speed=1.0, loop=False, kinds=("state",), clock=time.monotonic):
        self.session_dir, self.speed, self.loop, self.kinds, self.clock = session_dir, float(speed), loop, tuple(kinds), clock
        self.rows, self.i, self.lap, self.t_offset = None, 0, 0, 0
        self._t_start = self._t0 = None
        self.n_sent = 0

    def start(self):
        if self.rows is None:
            from .session import iter_rows                  # here, not at import: make_source("replay:...") must not touch disk
            self.rows = [r for r in iter_rows(self.session_dir) if r.get("kind") in self.kinds]
            if not self.rows:
                raise ValueError(f"{self.session_dir}: no rows of kind {self.kinds}")
            self._t0 = self.rows[0]["t_ms"]
            self.span_ms = self.rows[-1]["t_ms"] - self._t0
            self._t_start = self.clock()
        return self

    @property
    def done(self):
        return self.rows is not None and not self.loop and self.i >= len(self.rows)

    def _shifted(self, data):
        if self.t_offset and isinstance(data, dict) and "t" in data:
            data = dict(data, t=data["t"] + self.t_offset)
        return data

    def poll_kinds(self):
        if self.rows is None:
            self.start()
        out = []
        while True:
            if self.i >= len(self.rows):
                if not self.loop:
                    break
                self.lap += 1; self.i = 0
                self.t_offset += self.span_ms + self.LAP_GAP_MS
                self._t_start = self.clock()
            r = self.rows[self.i]
            if self.speed > 0 and (self.clock() - self._t_start) * 1000.0 * self.speed < r["t_ms"] - self._t0:
                break
            out.append((r["kind"], self._shifted(r["data"])))
            self.i += 1; self.n_sent += 1
            if self.i == len(self.rows) and self.loop:
                break                                           # one lap per poll at most (and the lap gap runs in real time)
        return out

    def poll(self):
        return [d for k, d in self.poll_kinds() if k == "state"]


class SimSource(PositioningSource):
    """In-process archive/sim/world.py World: poll() = world.poll_state(). The caller advances the world."""
    name = "sim"

    def __init__(self, world):
        self.world = world

    def poll(self):
        return list(self.world.poll_state())


class ListSource(PositioningSource):
    """Hand-fed queue for tests and for pushing synthetic fixes: src.push(msg); src.poll() -> [msg]."""
    def __init__(self, msgs=None, name="list"):
        self.queue, self.name = list(msgs or []), name

    def push(self, msg):
        self.queue.append(msg)

    def poll(self):
        out, self.queue = self.queue, []
        return out


class PlaceholderSource(PositioningSource):
    """Slot for the sensor that is not decided yet. Candidates: UWB anchors (DWM1001, ~10 cm xy), ToF ranging
    for altitude (VL53L0X is already on the gondola: telemetry `alt`), phone ARKit/ARCore pose, a mocap system.
    TODO(positioning): implement start/poll/stop producing PROTOCOL s4 dicts with src="<sensor>", set `name`,
    add it to REGISTRY, and give laptop/positioning/fuse.py its noise figure."""
    name = "todo"

    def __init__(self, *args, **kwargs):
        raise NotImplementedError("PlaceholderSource: sensor not chosen yet; see the docstring for how to fill it in")

    def poll(self):
        return []


REGISTRY = {"udp": UdpStateSource, "replay": ReplaySource, "sim": SimSource}


def make_source(spec, **kw):
    """spec = "udp" | "udp:<port>" | "replay:<session_dir>" | "sim" (needs world=...). Extra kwargs go to the class.
    Split once on ':' so Windows paths like replay:C:\\data\\positioning\\x survive."""
    kind, _, arg = str(spec).partition(":")
    if kind == "udp":
        return UdpStateSource(int(arg) if arg else STATE_PORT, **kw)
    if kind == "replay":
        if not arg:
            raise ValueError("replay source needs a session directory: replay:data/positioning/<session>")
        return ReplaySource(arg, **kw)
    if kind == "sim":
        if "world" not in kw:
            raise ValueError("sim source needs world=World(...)")
        return SimSource(kw["world"])
    raise ValueError(f"unknown source {spec!r}; known: {', '.join(REGISTRY)}")
