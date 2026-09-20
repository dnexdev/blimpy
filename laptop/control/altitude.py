"""The height loop, on its own: camera fixes in, lift-motor command out. Nothing else has a say in it: pilot.py,
follow_me.py and hover.py all take vz from here AFTER the behaviours / the voice agent have decided the rest, so
whatever Blimpy is doing sideways, the height is held the same way.

  lift = LiftHold()                      # gains: config.HOVER
  lift.update(balloon_xyz, t_s)          # every camera fix (world metres, seconds)
  lift.arm(z=None)                       # hold the height it has now (or z); fresh trim
  vz = lift.cmd(now, offset=0.0)         # every tick while armed; offset = "go up / down a bit" from the voice

Two ways to drive the fan (config.HOVER ONOFF): flat out or off (the motor is weak and the balloon is kept a little
heavy: the fan only ever has to push up), or AltHold's P + I + D. Either way a DROP is caught first: the moment the
balloon is seen sinking at or below its target the fan goes on, without waiting for the height error to build up or
for the minimum off time. The camera + Bluetooth + motor spin-up are ~0.5 s late; a balloon that was allowed to
gather speed downward took metres to stop (log hover3, 2026-09-20).
"""
from .. import config
from .protocol import clamp
from .follow_me import AltHold

G = config.HOVER
A = config.AVOID


class ZTrack:
    """Height and vertical speed straight from the camera fixes of the last VZ_WIN_S seconds: a line fitted through
    them, after dropping the ones far from their median (a wrong box for a frame or two). The shared estimator's
    vertical speed follows a change over several seconds (beta 0.02): fine for a smooth P+I+D, but an on/off fan
    driven by it kept braking a climb that had ended seconds ago (log hover3, 2026-09-20). Same .p / .v as the
    estimator, so AltHold takes it."""

    def __init__(self, win_s, gate_m):
        self.win, self.gate, self.pts, self.p, self.v, self.n_bad = win_s, gate_m, [], None, [0.0, 0.0, 0.0], 0

    def update(self, xyz, t):
        self.pts.append((t, xyz[2])); self.pts = [q for q in self.pts if t - q[0] <= self.win]
        zs = sorted(q[1] for q in self.pts); med = zs[len(zs) // 2]
        good = [q for q in self.pts if abs(q[1] - med) <= self.gate]
        self.n_bad = len(self.pts) - len(good)
        if len(good) < 3 or good[-1][0] - good[0][0] < 0.25 * self.win:
            self.p, self.v = [xyz[0], xyz[1], med], [0.0, 0.0, 0.0]; return
        tm = sum(q[0] for q in good) / len(good); zm = sum(q[1] for q in good) / len(good)
        den = sum((q[0] - tm) ** 2 for q in good)
        vz = sum((q[0] - tm) * (q[1] - zm) for q in good) / den if den > 1e-9 else 0.0
        self.p, self.v = [xyz[0], xyz[1], zm + vz * (t - tm)], [0.0, 0.0, vz]      # the line, read at the newest fix


class LiftHold:
    def __init__(self, gains=None, onoff=None):
        self.g = G if gains is None else gains
        self.onoff = self.g["ONOFF"] if onoff is None else onoff
        self.track = ZTrack(self.g["VZ_WIN_S"], self.g["Z_GATE_M"])        # height + the speed the switching line uses
        self.fast = ZTrack(self.g["DROP_WIN_S"], self.g["Z_GATE_M"])       # a shorter look, only to catch a drop early
        self.alt, self.z_hold = AltHold(self.g), None
        self.fan_on, self.t_switch, self.t_sat, self.note = False, -1e9, None, ""
        self.a_on, self.a_off, self._ph = self.g["A_ON0"], self.g["A_OFF0"], None      # m/s^2 with the fan on / off: learnt in flight (_learn)

    p = property(lambda self: self.track.p)
    v = property(lambda self: self.track.v)
    trim = property(lambda self: self.alt.i)

    def update(self, xyz, t):
        self.track.update(xyz, t); self.fast.update(xyz, t)

    def arm(self, z=None):
        """Hold z (default: the height it has now). False when there is no height yet."""
        if self.track.p is None: return False
        self.z_hold = clamp(self.track.p[2] if z is None else z, A["Z_MIN"], A["Z_MAX"])
        self.alt, self.fan_on, self.t_switch, self.t_sat = AltHold(self.g), False, -1e9, None    # a fresh trim: the last one belongs to another fill
        return True

    def _learn(self, now, vz):
        """Vertical acceleration of the running phase = change of the fitted speed over it, once the phase is LEARN_MIN_S old
        (the first LAT_S of a phase still belongs to the previous one). Slow average; kept on the right side of zero."""
        g = self.g
        if now - self.t_switch < g["LAT_S"] + g["VZ_WIN_S"]: return
        if self._ph is None: self._ph = (now, vz); return
        t0, v0 = self._ph
        if now - t0 >= g["LEARN_MIN_S"]:
            a = (vz - v0) / (now - t0); self._ph = (now, vz)
            if self.fan_on: self.a_on = clamp(self.a_on + g["LEARN_K"] * (a - self.a_on), g["A_MIN"], g["A_MAX"])
            else:           self.a_off = clamp(self.a_off + g["LEARN_K"] * (a - self.a_off), -g["A_MAX"], -g["A_MIN"])

    def target(self, offset=0.0):
        return None if self.z_hold is None else clamp(self.z_hold + offset, A["Z_MIN"], A["Z_MAX"])

    def cmd(self, now, offset=0.0):
        g, z_t = self.g, self.target(offset)
        if z_t is None or self.track.p is None:
            self.note = "no height"; return 0.0
        ez, vz_m = z_t - self.track.p[2], self.track.v[2]
        dropping = min(vz_m, self.fast.v[2]) < -g["DROP_V"] and ez > -g["DROP_MARGIN_M"]   # sinking, and not from well above the target
        if not self.onoff:
            u = self.alt.cmd(z_t, self.track, now)
            if abs(self.alt.i) >= g["Z_I_MAX"] - 1e-6:
                self.t_sat = now if self.t_sat is None else self.t_sat
            else:
                self.t_sat = None
            sat = self.t_sat is not None and now - self.t_sat > g["TRIM_SAT_S"]
            self.note = ("TRIM AT LIMIT: too " + ("heavy" if self.alt.i > 0 else "light")) if sat else ("catching a drop" if dropping else "holding")
            return max(u, g["ON_DUTY"]) if dropping else u
        # Flat out or off: WHEN to switch comes from where the balloon will end up, not from gains. Fan off and rising, it
        # coasts to z + vz^2 / (2 a_off); fan on and sinking, it stops at z - vz^2 / (2 a_on). Both are read LAT_S ahead
        # (camera + Bluetooth + spin-up), and a_off / a_on are measured in flight from every phase, so a leaking balloon
        # (sinks faster by the minute) or a fresh battery (fan stronger) moves the switching points by itself.
        self._learn(now, vz_m)
        a = self.a_on if self.fan_on else self.a_off
        z_p = self.track.p[2] + vz_m * g["LAT_S"] + 0.5 * a * g["LAT_S"] ** 2; v_p = vz_m + a * g["LAT_S"]
        if v_p > 0:   end = z_p + v_p * v_p / (2 * max(1e-3, -self.a_off))          # coasting up with the fan off: the top of the arc
        else:         end = z_p - v_p * v_p / (2 * max(1e-3, self.a_on))            # sinking: where the fan can stop it
        want = True if end < z_t - g["Z_DEADBAND"] else False if end > z_t + g["Z_DEADBAND"] else self.fan_on
        if dropping: want = True
        if want != self.fan_on and (want or now - self.t_switch >= g["ONOFF_MIN_S"]):   # ON is never delayed; OFF waits for the motor to have
            self.fan_on, self.t_switch, self._ph = want, now, None                      # reached full duty (the mixer slews, ~0.4 s)
        self.note = ("fan ON (drop)" if dropping else "fan ON") if self.fan_on else "fan off"
        return g["ON_DUTY"] if self.fan_on else 0.0
