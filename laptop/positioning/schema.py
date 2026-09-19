"""Positioning log rows and fixes: the shapes shared by the recorder, the replayer, the loader and the sources.

Log format (data/positioning/<session>/log.jsonl), one JSON object per line:
  {"kind": "state"|"telem"|"cmd"|"truth"|"event", "t_ms": <recorder clock>, "wall": <time.time()>, "data": {...}}
  state  data = PROTOCOL.md s4 datagram verbatim   {"t","balloon":[x,y,z]|null,"person":[x,y,z]|null,"person_id","src",...}
  telem  data = PROTOCOL.md s3 datagram verbatim   {"t","yaw","yr","pitch","roll","alt","vbat","armed","age","mL","mR","mS","mV"}
  cmd    data = PROTOCOL.md s2 datagram verbatim   {"t","vf","vs","yr","vz","arm"}
  truth  data = laptop/sim/world.py World.truth (+ "t"): ground truth, simulator only
  event  data = {"text": "arm", ...any keyword fields}  markers: arm/disarm, intents, notes
Never re-key upstream messages here: a producer may add fields and old logs must still load (session.load is generic).

Clocks: data.t is the SENDER's clock and differs per kind (ESP32 millis, vision protocol.now_ms(), sim epoch).
t_ms = protocol.now_ms() and wall = time.time() are the RECORDER's, identical for every kind in one session:
use them to align kinds within a session and `wall` to join sessions recorded by two processes at once
(fake_esp32 --log writes truth, follow_me --log writes what it consumed and commanded).
"""
import time
from dataclasses import dataclass
from ..control.protocol import now_ms

SCHEMA_VERSION = 1
KINDS = ("state", "telem", "cmd", "truth", "event")
OBJECTS = ("balloon", "person")          # the things a state message positions


def row(kind, data, t_ms=None, wall=None):
    """One log line. `data` is stored as given (verbatim); t_ms / wall default to the recorder's clocks now."""
    if kind not in KINDS:
        raise ValueError(f"unknown row kind {kind!r}; expected one of {KINDS}")
    return {"kind": kind, "t_ms": now_ms() if t_ms is None else int(t_ms),
            "wall": time.time() if wall is None else float(wall), "data": data}


@dataclass
class Fix:
    """One positioned object at one instant, in the world frame (PROTOCOL.md s5). The unit every source speaks.
    quality: None = unknown; sources that can, put something monotonic here (reprojection error, box size, ...).
    TODO(positioning): decide the quality convention (px error? covariance? 0..1?) once the sensor set is known."""
    t_ms: int
    xyz: tuple | None
    source: str = "?"
    quality: float | None = None

    @property
    def seen(self):
        return self.xyz is not None


def fix_from_state(msg, obj="balloon"):
    """PROTOCOL s4 message -> Fix for `obj` ("balloon" | "person"), or None if the message does not carry it."""
    if obj not in OBJECTS:
        raise ValueError(f"obj must be one of {OBJECTS}, got {obj!r}")
    p = msg.get(obj)
    if p is None:
        return None
    return Fix(int(msg.get("t", 0)), (float(p[0]), float(p[1]), float(p[2])), str(msg.get("src", "?")))
