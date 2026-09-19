"""Balloon state estimator (deliberately simple, no EKF).

position / velocity : alpha-beta filter (steady-state constant-velocity Kalman) on the triangulated position.
                      The prediction step uses the acceleration we EXPECT from what we are commanding (known
                      thrust, drag), so the filter can be smooth (low beta) without lagging our own manoeuvres.
                      A jump gate stops a single bad triangulation from throwing the velocity.
heading             : gyro yaw from telemetry + an offset. The offset is learned from HOW THE BALLOON RESPONDS
                      to a push: the commanded thrust is known in the body frame, the resulting change of velocity
                      is measured in the world frame, and the angle between them is the heading error. Works during
                      any deliberate manoeuvre (nudge, follow, strafe, twitch), immune to a constant wind.
                      Learning uses a second, measurement-only velocity track (the predicted one would just agree
                      with the command), a command history delayed by the sensing/actuation latency, and only
                      trusts a push once enough thrust-time has accumulated for the response to beat vision noise.
gyro bias           : the slow drift of that offset between manoeuvres IS the gyro bias; it is fitted from the
                      offset at the end of successive pushes, fed forward, and exposed (`bias_hat`) so the
                      controller can hold a true zero yaw rate.
height              : the downward ToF on the gondola (telemetry `alt`) owns z while its readings are fresh and
                      agree with the estimate (update_telem); a hand / table / person under the lens is rejected by a
                      jump gate and vision z takes over. No ToF (alt = -1): vision z as before.
"""
import cmath, math, time
from collections import deque
from .. import config
from .protocol import wrap

# Physical constants: config.PHYS is the single source (laptop/sim/world.py::Balloon reads the same numbers).
# Rough is fine here, they only shape the prediction. M_EFF / K_DRAG follow from D exactly as in Balloon.__init__.
_V = math.pi / 6 * config.PHYS["D"] ** 3
M_EFF = 0.05 + config.PHYS["M_GONDOLA"] + (0.166 + 0.5 * 1.2) * _V   # kg, skin + gondola + helium + added mass (~0.83)
K_DRAG = 0.5 * 1.2 * 0.47 * math.pi / 4 * config.PHYS["D"] ** 2      # N s^2/m^2 (~0.27)
T_MAX = config.PHYS["T_MAX"]        # N per motor at duty 1
REV_EFF = config.PHYS["REV_EFF"]
RESPONSE_DELAY = 0.30       # s from a command to its effect showing in the vision fixes (slew + spin-up + latency)
TOF_BELOW = config.PHYS["TOF_BELOW"]   # m, VL53L0X lens below the balloon centre: centre z = alt + TOF_BELOW
TOF_MIN_M, TOF_MAX_M = 0.05, 2.0       # plausible reading (firmware sends -1 beyond 2000 mm; < 5 cm = covered / on the floor)


def _acc(duty, n_motors):
    """Body-axis acceleration (m/s^2) produced by `n_motors` at signed `duty`."""
    a = n_motors * T_MAX / M_EFF * duty * abs(duty)
    return a if duty >= 0 else a * REV_EFF


class StateEstimator:
    def __init__(self, alpha=0.3, beta=0.05, alpha_z=0.25, beta_z=0.02, alpha_m=0.3, beta_m=0.05,
                 k_head=0.3, tau_gate=0.5, tau_z=3.0, a_min=0.05, s_min=0.12, z_thresh=0.006, jump_m=0.6,
                 tof_jump_m=0.3, tof_fresh_s=0.3):
        self.alpha, self.beta, self.alpha_z, self.beta_z = alpha, beta, alpha_z, beta_z
        self.tof_jump_m, self.tof_fresh_s = tof_jump_m, tof_fresh_s
        self.alpha_m, self.beta_m = alpha_m, beta_m
        self.k_head, self.tau_gate, self.tau_z = k_head, tau_gate, tau_z
        self.a_min, self.s_min, self.z_thresh, self.jump_m = a_min, s_min, z_thresh, jump_m
        self.p = None
        self.v = [0.0, 0.0, 0.0]
        self.t = None
        self.yaw_gyro = None
        self.yr_meas = 0.0
        self.offset = 0.0
        self.head_ok = False
        self.bias_hat = 0.0
        self.bias_ok = False             # True once the bias has been fitted from >= 3 pushes
        self.contact = False             # set by the caller when the envelope is (nearly) touching something
        # ToF altimeter (telemetry alt)
        self.alt_raw = None              # last telemetry alt as received (m, -1 = none), or None
        self.alt_rejects = 0             # readings refused by the gate (hand / table / person under the lens)
        self._z_tof = None               # last ACCEPTED reading as balloon-centre height
        self._t_alt = None               # loop-clock time of that acceptance (freshness)
        self._t_alt_ms = None            # the board's t of the last valid reading (dt inside the ToF stream)
        self._t_now = None               # latest loop-clock time seen (observe_motion / update_telem)
        # command model
        self._a = 0j                     # lagged commanded acceleration, world frame (complex), for prediction
        self._az = 0.0
        self._hist = deque()             # (t, a_body complex, yaw_gyro) for the delayed learner input
        self._t_motion = None
        # learner
        self._pm = None; self._vm = [0.0, 0.0]      # measurement-only track
        self._ag = 0j                    # gate: delayed + smoothed commanded acceleration
        self._z = 0j                     # correlation accumulator  sum(dv * conj(a))
        self._S = 0.0                    # accumulated |a| dt of the current push (m/s of expected response)
        self._A2 = 0.0                   # accumulated |a|^2 dt: what |z| would be if the balloon answered the push perfectly
        self._C2 = 0j                    # accumulated a^2 dt: |C2|/A2 = 1 when the push stays on ONE axis (push + return ok)
        self._A1 = 0j                    # accumulated a dt: the push's mean direction (to detect a sharp change of direction)
        self._blank_until = -1e9         # after a direction change the lagging response still belongs to the OLD direction
        self._push = False; self._push_end = -1e9; self._push_corr = 0; self._t_spin = -1e9; self._v0 = 0.0
        self._push_conv = False
        self.lessons_total = 0           # every correction ever applied (callers diff it to see if a push taught)
        self.head_confident = False      # True once a strong, clean push from rest has confirmed the heading
        self._t_corr = -1e9
        self._off_unwrapped = 0.0
        self._anchors = []               # (t, unwrapped offset) after pushes, for the bias fit
        self._anchor_push = -1; self._push_id = 0
        self._reject = 0

    # ------------------------------------------------------------------ vision
    def update_balloon(self, p, t_ms):
        p = [float(x) for x in p]
        if self.p is None:
            self.p, self.t = p, t_ms
            if self.alt_ok and abs(self._z_tof - p[2]) < self.tof_jump_m:
                self.p[2] = self._z_tof                     # the ToF cannot give xy, but its z is the better one
            self._pm = p[:2]
            return
        dt = min(0.5, max(1e-3, (t_ms - self.t) / 1000.0))
        sp = math.hypot(self.v[0], self.v[1])
        a = [self._a.real - K_DRAG / M_EFF * sp * self.v[0],
             self._a.imag - K_DRAG / M_EFF * sp * self.v[1],
             self._az - K_DRAG / M_EFF * abs(self.v[2]) * self.v[2]]
        pred_p = [q + v * dt + 0.5 * ai * dt * dt for q, v, ai in zip(self.p, self.v, a)]
        pred_v = [v + ai * dt for v, ai in zip(self.v, a)]
        tof = self.alt_ok
        if tof:                                          # the ToF stream predicts and corrects z on its own dt
            pred_p[2], pred_v[2] = self.p[2], self.v[2]
        res = [z - q for z, q in zip(p, pred_p)]
        if math.hypot(res[0], res[1]) > self.jump_m and self._reject < 3:
            self._reject += 1               # one bad fix: ignore. Three in a row: it really moved, accept.
            return
        self._reject = 0
        az, bz = (0.0, 0.0) if tof else (self.alpha_z, self.beta_z)
        ab = (self.alpha, self.alpha, az); bb = (self.beta, self.beta, bz)
        self.p = [q + al * r for q, r, al in zip(pred_p, res, ab)]
        self.v = [v + be / dt * r for v, r, be in zip(pred_v, res, bb)]
        self.t = t_ms
        # measurement-only track (no command prediction) for the heading learner
        vm_old = list(self._vm)
        pm_pred = [q + v * dt for q, v in zip(self._pm, self._vm)]
        rm = [z - q for z, q in zip(p[:2], pm_pred)]
        self._pm = [q + self.alpha_m * r for q, r in zip(pm_pred, rm)]
        self._vm = [v + self.beta_m / dt * r for v, r in zip(self._vm, rm)]
        self._learn_heading(vm_old, dt)

    # ------------------------------------------------------------------ telemetry
    def update_telem(self, yaw, yr=0.0, alt=None, t_ms=None, now=None):
        """One telemetry frame (PROTOCOL.md s3). alt = VL53L0X range in m (-1 / None = no sensor). t_ms = the board's own
        t, used ONLY for dt between its frames. now = loop clock (the one observe_motion gets); default = last seen.
        ToF freshness is judged on `now`, never on t_ms: ESP32 millis, vision t and the loop clock are three epochs."""
        self.yaw_gyro, self.yr_meas = float(yaw), float(yr)
        if now is not None:
            self._t_now = now
        if alt is None:
            return
        self.alt_raw = float(alt)
        if not (TOF_MIN_M < self.alt_raw < TOF_MAX_M):
            return                                       # -1: no sensor / timeout / beyond range
        z = self.alt_raw + TOF_BELOW
        dt = 0.05 if (t_ms is None or self._t_alt_ms is None) else min(0.2, max(0.02, (t_ms - self._t_alt_ms) / 1000.0))
        self._t_alt_ms = t_ms
        if self.p is not None:
            az = self._az - K_DRAG / M_EFF * abs(self.v[2]) * self.v[2]
            pz = self.p[2] + self.v[2] * dt + 0.5 * az * dt * dt
            pv = self.v[2] + az * dt
            r = z - pz
            if abs(r) > self.tof_jump_m:                 # not the floor under the lens (hand, table, person): vision keeps z;
                self.alt_rejects += 1                    # the ToF is trusted again only once it agrees with the estimate
                return
            self.p[2] = pz + self.alpha_z * r
            self.v[2] = pv + self.beta_z / dt * r
        self._z_tof, self._t_alt = z, self._t_now

    @property
    def alt_ok(self):
        """True while a ToF reading was accepted within tof_fresh_s (loop clock): z is ToF-driven, vision z ignored."""
        return self._t_alt is not None and self._t_now is not None and self._t_now - self._t_alt < self.tof_fresh_s

    @property
    def z_src(self):
        return "tof" if self.alt_ok else "cam"

    @property
    def psi(self):
        return None if self.yaw_gyro is None else wrap(self.yaw_gyro + self.offset)

    @property
    def speed(self):
        return math.hypot(self.v[0], self.v[1])

    # ------------------------------------------------------------------ commands
    def observe_motion(self, vf_cmd, vs_cmd=0.0, vz_cmd=0.0, now=None):
        """Call every tick with the duties being sent. Keeps the expected acceleration (world frame, using the
        CURRENT heading estimate) for the filter's prediction, a delayed body-frame history for the learner,
        and applies the gyro-bias feed-forward to the offset."""
        now = time.monotonic() if now is None else now
        dt = 0.0 if self._t_motion is None else min(0.5, max(0.0, now - self._t_motion))
        self._t_motion = now
        self._t_now = now
        k = min(1.0, dt / 0.15)
        self._az += (_acc(vz_cmd, 1) - self._az) * k
        if self.yaw_gyro is None:
            return
        self._bump_offset(-self.bias_hat * dt)
        ab = complex(_acc(vf_cmd, 2), _acc(vs_cmd, 1))
        self._a += (ab * cmath.exp(1j * self.psi) - self._a) * k
        self._hist.append((now, ab, self.yaw_gyro))
        while self._hist and now - self._hist[0][0] > RESPONSE_DELAY + 1.0:
            self._hist.popleft()

    def _a_delayed(self):
        """Commanded acceleration RESPONSE_DELAY ago, in the world frame under the current offset."""
        if not self._hist or self._t_motion is None:
            return 0j
        t_want = self._t_motion - RESPONSE_DELAY
        pick = self._hist[0]
        for h in self._hist:
            if h[0] <= t_want: pick = h
            else: break
        return pick[1] * cmath.exp(1j * (pick[2] + self.offset))

    # ------------------------------------------------------------------ heading learning
    def _bump_offset(self, d):
        self.offset = wrap(self.offset + d)
        self._off_unwrapped += d

    def _learn_heading(self, vm_old, dt):
        if self.yaw_gyro is None:
            return
        now = self._t_motion
        if abs(self.yr_meas) > 0.25 or self.contact:        # spinning or pushing on a wall: the response is not ours
            self._t_spin = now
        if now - self._t_spin < 1.5:
            self._ag, self._z, self._S, self._push = 0j, 0j, 0.0, False
            return
        self._ag += (self._a_delayed() - self._ag) * min(1.0, dt / self.tau_gate)
        a = self._ag
        decay = math.exp(-dt / self.tau_z)
        if abs(a) < self.a_min:
            self._z *= decay; self._S *= decay; self._A2 *= decay; self._C2 *= decay
            if self._push:
                self._push, self._push_end = False, now
            return
        if not self._push:
            self._push = True
            if now - self._push_end > 2.0:                  # a new push, not a continuation: start clean
                self._new_window(now)
        elif abs(self._A1) > 0.05 and math.cos(cmath.phase(a) - cmath.phase(self._A1)) < 0.5:
            # the push turned by more than 60 deg (e.g. twitch, then the hold pulling back another way): the velocity
            # response we are about to see still belongs to the old direction -> new window, and skip its first 1.2 s
            self._new_window(now)
            self._blank_until = now + 1.0
        self._A1 += a * dt
        if now < self._blank_until:
            return
        self._S += abs(a) * dt
        v = complex(self._vm[0], self._vm[1])
        dv = v - complex(vm_old[0], vm_old[1])
        dv += K_DRAG / M_EFF * abs(v) * v * dt                   # add the drag back: what thrust alone did
        self._z = self._z * decay + dv * a.conjugate()
        self._A2 = self._A2 * decay + abs(a) ** 2 * dt
        self._C2 = self._C2 * decay + a * a * dt
        # steady cruising (thrust just balancing drag) says little about heading and is biased by any air movement:
        # require the response to be a real fraction of what the push should have produced (any direction: |z|).
        # Dithering (hover corrections changing direction faster than the vision track can follow) pairs velocity
        # changes with the wrong command: require the push to have stayed on one axis (|C2|/A2 near 1).
        if self._S < self.s_min or abs(self._z) < self.z_thresh or abs(self._z) < 0.3 * self._A2                 or abs(self._C2) < 0.75 * self._A2:
            return
        d = cmath.phase(self._z)
        clean = min(1.0, abs(self._z) / (0.55 * self._A2))    # 1 = the velocity really changed as pushed (not cruising)
        w = 1.0 if self._v0 < 0.35 else 0.3                   # lessons taken while cruising: less weight (wind bias)
        if self._push_conv:
            w *= 0.15                                         # this push already converged: only track slowly, don't
        step = self.k_head * min(1.0, self._S / 0.3) * clean * w * d   # random-walk on its noise for the rest of it
        self._bump_offset(step)
        self._z *= cmath.exp(-1j * step)
        self._a *= cmath.exp(1j * step)
        self._ag *= cmath.exp(1j * step)
        if not self.head_ok:
            self.head_ok, self._t_corr = True, now
        self._push_corr += 1
        self.lessons_total += 1
        if self._push_corr >= 3 and abs(d) < 0.12:
            self._push_conv = True
        if self._S >= 0.2 and clean >= 1.0 and self._push_corr >= 3 and self._v0 < 0.35 and abs(d) < 0.12:
            self._t_corr = now                              # a real, clean push from rest, converged: fresh + anchor
            self.head_confident = True
            self._anchor(now)                               # (ignored unless the last anchor is >= 10 s old)

    def _new_window(self, now):
        self._z, self._S, self._A2, self._C2, self._A1, self._push_corr = 0j, 0.0, 0.0, 0j, 0j, 0
        self._push_conv = False
        self._push_id += 1
        self._v0 = math.hypot(*self._vm)                    # speed at push start: lessons at speed are wind-biased

    def _anchor(self, now):
        """One anchor per push, holding the offset as of the LAST correction of that push (i.e. converged)."""
        if self._anchor_push == self._push_id:
            self._anchors[-1] = (now, self._off_unwrapped)
        elif self._anchors and now - self._anchors[-1][0] < 10.0:
            return
        else:
            self._anchors.append((now, self._off_unwrapped)); self._anchor_push = self._push_id
        self._anchors = [(t, o) for t, o in self._anchors if now - t < 150.0]
        span = now - self._anchors[0][0]
        if len(self._anchors) < 3 or span < 45.0:
            return
        gain = 0.5
        n = len(self._anchors)                                  # least-squares slope of offset vs time
        mt = sum(t for t, _ in self._anchors) / n; mo = sum(o for _, o in self._anchors) / n
        sxx = sum((t - mt) ** 2 for t, _ in self._anchors)
        sxy = sum((t - mt) * (o - mo) for t, o in self._anchors)
        slope = sxy / sxx if sxx > 0 else 0.0
        self.bias_hat = max(-0.015, min(0.015, self.bias_hat + gain * (-slope - self.bias_hat)))
        self.bias_ok = len(self._anchors) >= 3 and span >= 45.0

    def forget_heading(self):
        """Operator asked for a fresh heading acquisition (or the gondola was re-hung)."""
        self.head_ok, self.head_confident, self._t_corr, self._z, self._S, self._A2 = False, False, -1e9, 0j, 0.0, 0.0
        self.bias_hat, self.bias_ok, self._anchors = 0.0, False, []

    @property
    def push_lessons(self):
        """Corrections applied during the current push (0 = it has taught nothing yet)."""
        return self._push_corr if self._push else 0

    @property
    def push_strength(self):
        """Accumulated thrust-time of the current push (m/s); the learner trusts a push from ~0.2 up."""
        return self._S if self._push else 0.0

    @property
    def heading_fresh_s(self):
        """Seconds since the heading was last confirmed by a manoeuvre (large = trust it less)."""
        return 1e9 if self._t_motion is None else self._t_motion - self._t_corr
