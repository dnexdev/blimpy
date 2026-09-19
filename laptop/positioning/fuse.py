"""Merge several PositioningSources into ONE state stream. STUB: newest-by-priority per object, no estimation yet.

  from laptop.positioning.fuse import Fuser
  from laptop.positioning.sources import make_source
  f = Fuser([make_source("udp"), make_source("udp:5008")], priority=["udp", "udp:5008"], max_age_ms=500)
  with f:
      for msg in f.poll():          # 0 or 1 PROTOCOL s4 message per call, src="fused", srcs={"balloon": ..., "person": ...}
          ...

Rule per object (balloon, person): among sources that have reported it, the highest-priority one whose latest fix
is fresher than max_age_ms wins; if none is fresh, the most recently received one. Freshness is judged on the
RECEIPT clock (protocol.now_ms), never on data.t: sources run on different clocks (ESP32 millis, vision, sim).
The output t is the t of the source that supplied the balloon, because the estimator only looks at balloon t
differences; switching balloon sources mid-stream therefore produces one wrong dt.

TODO(positioning) - real fusion once the sensor set is known:
  - per-source noise: vision stereo ~2-4 cm (reprojection px -> m), mono ~5-10 % of range, ToF alt ~2 cm (z only),
    UWB ~10 cm xy;  weighted average when several are fresh instead of winner-takes-all
  - jump gate like laptop/control/estimator.py::StateEstimator (jump_m) so one bad source cannot yank the fix
  - per-source clock offset estimation so t can be aligned instead of copied from the winner
  - ToF altitude: DONE in laptop/control/estimator.py::update_telem (it rides the telemetry link, not a PositioningSource)
  - person_id continuity across sources (ids are per-tracker today)
"""
from ..control.protocol import now_ms
from .schema import OBJECTS
from .sources import PositioningSource


class Fuser(PositioningSource):
    name = "fused"

    def __init__(self, sources, priority=None, max_age_ms=500, clock=now_ms):
        self.sources = list(sources)
        self.names = []
        for s in self.sources:                                   # unique names even if two UdpStateSources are "udp"
            n = s.name
            while n in self.names:
                n += "'"
            self.names.append(n)
        rank = {n: i for i, n in enumerate(priority or self.names)}
        self.rank = [rank.get(n, len(rank)) for n in self.names]   # unknown names: lowest priority
        self.max_age_ms, self.clock = max_age_ms, clock
        self.latest = {}                                          # (source index, object) -> (t_received, msg)

    def start(self):
        for s in self.sources: s.start()
        return self

    def stop(self):
        for s in self.sources: s.stop()

    def _pick(self, obj, now):
        cands = [(i, t, m) for (i, o), (t, m) in self.latest.items() if o == obj]
        if not cands:
            return None
        fresh = [c for c in cands if now - c[1] <= self.max_age_ms]
        if fresh:
            return min(fresh, key=lambda c: (self.rank[c[0]], -c[1]))
        return max(cands, key=lambda c: c[1])

    def poll(self):
        now = self.clock()
        new = False
        for i, s in enumerate(self.sources):
            for msg in s.poll():
                new = True
                for obj in OBJECTS:
                    if msg.get(obj) is not None:
                        self.latest[(i, obj)] = (now, msg)
        if not new:
            return []
        out = {"t": None, "balloon": None, "person": None, "person_id": -1, "src": "fused", "srcs": {}}
        for obj in OBJECTS:
            pick = self._pick(obj, now)
            if pick is None:
                continue
            i, t_recv, msg = pick
            if now - t_recv > self.max_age_ms:
                continue                                          # everything stale: report "not seen" honestly
            out[obj] = msg[obj]
            out["srcs"][obj] = self.names[i]
            if obj == "balloon":
                out["t"] = msg.get("t")
            if obj == "person":
                out["person_id"] = msg.get("person_id", -1)
        if out["t"] is None:
            out["t"] = now
        return [out]
