"""Simulated world for Blimpy: balloon physics, wind, a person, walls/obstacles and SENSOR models
(vision latency + noise + dropouts, telemetry/command latency + loss, gyro bias). No sockets, no wall clock:
`fake_esp32.py` drives it in real time over UDP, `archive/tools/scenarios.py` drives it headless, faster than real time.

The vehicle numbers (diameter, thrust, gondola mass, motor spacing, ToF offset) live in laptop/config.py PHYS, shared
with the estimator; `Balloon` reads them. Sensor/disturbance numbers are in REAL (IDEAL turns them all off).
"""
import math, random
from laptop import config
from laptop.control.protocol import FAILSAFE_MS, YR_MAX, clamp, mix, wrap

MIX_DT = 0.02        # firmware mixer tick (50 Hz)
PHYS_DT = 0.01       # physics sub-step

REAL = dict(
    # vision (two cameras + YOLO + triangulation on the laptop)
    vision_hz=15, vision_latency=0.12, vision_jitter=0.02,
    balloon_noise=0.02, z_noise=0.03, person_noise=0.04,
    person_drop_p=0.02, person_drop_s=(0.2, 1.0),      # per-frame chance a dropout starts, and how long it lasts
    balloon_drop_p=0.004, balloon_drop_s=(0.1, 0.4),
    person_outlier_p=0.005,                             # single-frame false detection 1-2 m away
    # links (laptop hotspot)
    telem_hz=20, telem_latency=0.03, telem_loss=0.02, cmd_latency=0.025, cmd_loss=0.02,
    # MPU6050 after a 2 s boot calibration: residual bias + noise
    gyro_bias=0.004, gyro_noise=0.003,
    # VL53L0X on the gondola looking down: 2 cm noise, occasional timeout (-1), a false short reading now and then.
    # tof=False here on purpose (and the 2026-09-19 box has no ultrasonic; ble_gondola --fake --tof adds one); the suite keeps it off so
    # seeded runs stay bit-identical to before (the V motor's tilt term leaks a little thrust into xy, and that is enough
    # to flip seed-marginal detours: python archive/tools/scenarios.py --tof shows which). ToF scenarios pass tof=True.
    tof=False, tof_noise=0.02, tof_drop_p=0.03, tof_false_p=0.003, tof_max=2.0, tof_bias=0.0,
    # room: HVAC gusts (Ornstein-Uhlenbeck), lift changing as the room warms / helium leaks
    gust_rms=0.06, gust_tau=4.0, lift_drift_n=0.006, lift_drift_period=240.0,
    # hardware imperfections: motor-to-motor thrust spread, prop spin-up, S motor not exactly through the centre, and the
    # SHARED motor supply (seen 2026-09-20: switching on more motors slows the others): the voltage at the motors drops by
    # supply_sag per unit of total duty, thrust goes with the square of it (all four at 0.4 -> -24 % volts, -42 % thrust)
    motor_gain=(1.0, 0.92, 1.05, 0.96), motor_tau=0.06, s_yaw_arm=0.03, supply_sag=0.15,
    # the eye on the balloon (laptop/vision/fpv.py): YOLO on the gondola camera's stream, 10 Hz, ~250 ms behind; range from
    # the person's height in the frame (~8 %), unknown (box cut) closer than fpv_cut_m. fpv=False here so the older scenarios
    # stay bit-identical; the eye scenarios and the BLE bridge's simulated robot turn it on.
    fpv=False, fpv_hz=10, fpv_latency=0.25, fpv_hfov=62.0, fpv_vfov=48.0, fpv_pitch=15.0, fpv_bearing_noise=0.02,
    fpv_range_noise=0.08, fpv_drop_p=0.03, fpv_cut_m=1.1,
    vision=True,          # False = no room camera at all (no balloon / person fixes on 5007): eye + altimeter only
)
IDEAL = dict(REAL, vision_latency=0.0, vision_jitter=0.0, balloon_noise=0.0, z_noise=0.0, person_noise=0.0,
             person_drop_p=0.0, balloon_drop_p=0.0, person_outlier_p=0.0, telem_latency=0.0, telem_loss=0.0,
             cmd_latency=0.0, cmd_loss=0.0, gyro_bias=0.0, gyro_noise=0.0, tof_noise=0.0, tof_drop_p=0.0, tof_false_p=0.0, gust_rms=0.0, lift_drift_n=0.0,
             motor_gain=(1.0, 1.0, 1.0, 1.0), motor_tau=0.0, s_yaw_arm=0.0, supply_sag=0.0,
             fpv_latency=0.0, fpv_bearing_noise=0.0, fpv_range_noise=0.0, fpv_drop_p=0.0)


class Balloon:
    """Force-based model of the real vehicle. Numbers come from the parts list.

    Envelope  GALPADA 48" latex inflated to D = 1.1 m. Displaced air 0.84 kg, helium ~0.12 kg, skin 0.05 kg,
              gondola 0.25 kg. A sphere accelerating through air also drags along HALF the displaced air
              ("added mass"), so what resists acceleration is ~0.83 kg even though it weighs ~nothing on a scale.
    Drag      0.5*rho*Cd*A*v^2, Cd 0.47 (sphere), A 0.95 m^2. Quadratic => it coasts a LONG way at low speed.
    Motors    4x 8520 coreless + 75 mm prop, ~50 gf flat out at 3.7 V, thrust ~ duty^2, ~12 gf at the 0.5 cap.
              A fixed-pitch prop spinning BACKWARDS only gives ~REV_EFF of that. Spin-up ~60 ms.
              L, R : rear, pointing forward, MOTOR_SPACING apart (differential = yaw)
              S    : sideways through the centre of the gondola (strafe / kill sideways drift)
              V    : vertical through the centre
    Yaw       torque = (T_R - T_L) * half the motor spacing; inertia ~0.025 kg m^2; ~no aerodynamic damping.
    Tilt      the motors sit ARM_BELOW m under the centre of buoyancy. Horizontal thrust swings the gondola forward
              (pendulum) until gravity on the gondola balances it, tilting the thrust axis UP by theta ~ 0.7*Fh/(Mg*g).
              At full push that is ~4 deg: ~1 gf of extra lift, and the V motor's thrust leans back a little.
    Lift      FREE_LIFT_N after ballast trim. Slightly NEGATIVE (heavy) is the safe trim: with power lost it
              settles to the floor instead of the ceiling, and the vertical motor mostly pushes UP (its efficient way).
    """
    D, RHO, CD = config.PHYS["D"], 1.2, 0.47                 # envelope diameter (config.PHYS), air density, sphere Cd
    M_SKIN, M_GONDOLA, RHO_HE = 0.05, config.PHYS["M_GONDOLA"], 0.166
    T_MAX = config.PHYS["T_MAX"]    # N per motor at duty 1.0 (~50 gf)
    REV_EFF = config.PHYS["REV_EFF"]  # reverse thrust fraction
    MOTOR_SPACING = config.PHYS["MOTOR_SPACING"]  # m between the L and R motors
    I_YAW, C_YAW = 0.025, 0.002     # kg m^2, N m s/rad
    ARM_BELOW = config.PHYS["ARM_BELOW"]  # m, motors below the centre of buoyancy
    TOF_BELOW = config.PHYS["TOF_BELOW"]  # m, ToF lens below the centre
    FREE_LIFT_N = -0.010            # -1 gf (slightly heavy)
    G = 9.81

    def __init__(self, x=0.0, y=0.0, z=1.5, psi=0.8, motor_gain=(1, 1, 1, 1), motor_tau=0.0, s_yaw_arm=0.0, supply_sag=0.0):
        V = math.pi / 6 * self.D ** 3
        self.m = self.M_SKIN + self.M_GONDOLA + self.RHO_HE * V + 0.5 * self.RHO * V   # ~0.83 kg
        self.k = 0.5 * self.RHO * self.CD * math.pi / 4 * self.D ** 2                  # ~0.27 N s^2/m^2
        self.x, self.y, self.z, self.psi = x, y, z, psi
        self.vx = self.vy = self.vz = self.w = 0.0
        self.gain, self.tau, self.s_yaw_arm, self.sag = motor_gain, motor_tau, s_yaw_arm, supply_sag
        self.duty = [0.0, 0.0, 0.0, 0.0]        # actual (spun-up) duties
        self.tilt = 0.0
        self.volts = 1.0                        # supply at the motors, fraction of nominal (the shared-battery sag)

    DUTY_START = 0.06               # coreless motor + DRV8833 do not turn below this duty

    def thrust(self, duty):
        if abs(duty) < self.DUTY_START:
            return 0.0
        t = self.T_MAX * duty * abs(duty)
        return t if duty >= 0 else t * self.REV_EFF

    @property
    def v(self):
        return math.hypot(self.vx, self.vy)

    def step(self, cmd, dt, wind=(0.0, 0.0), lift_extra=0.0):
        """cmd = commanded duties (mL, mR, mS, mV). Semi-implicit Euler; call with dt <= 0.02."""
        a = 1.0 if self.tau <= 0 else min(1.0, dt / self.tau)
        for i in range(4):
            self.duty[i] += (cmd[i] - self.duty[i]) * a
        # shared supply: every running motor pulls the voltage down for all of them (thrust ~ (volts x duty)^2)
        self.volts = max(0.3, 1.0 - self.sag * sum(abs(d) for d in self.duty))
        TL, TR, TS, TV = (self.thrust(d * self.volts) * g for d, g in zip(self.duty, self.gain))
        c, s = math.cos(self.psi), math.sin(self.psi)
        # horizontal thrust in the world frame (body +y = left = (-s, c))
        hx = (TL + TR) * c - TS * s
        hy = (TL + TR) * s + TS * c
        Fh = math.hypot(hx, hy)
        self.tilt = math.atan(0.7 * Fh / (self.M_GONDOLA * self.G))
        ct, st = math.cos(self.tilt), math.sin(self.tilt)
        ux, uy = (hx / Fh, hy / Fh) if Fh > 1e-9 else (0.0, 0.0)
        rvx, rvy = self.vx - wind[0], self.vy - wind[1]
        sp = math.hypot(rvx, rvy)
        fx = hx * ct - TV * st * ux - self.k * sp * rvx
        fy = hy * ct - TV * st * uy - self.k * sp * rvy
        fz = TV * ct + Fh * st + self.FREE_LIFT_N + lift_extra - self.k * abs(self.vz) * self.vz
        self.vx += fx / self.m * dt; self.vy += fy / self.m * dt; self.vz += fz / self.m * dt
        torque = (TR - TL) * self.MOTOR_SPACING / 2 + TS * self.s_yaw_arm - self.C_YAW * self.w
        self.w += torque / self.I_YAW * dt
        self.psi = wrap(self.psi + self.w * dt)
        self.x += self.vx * dt; self.y += self.vy * dt; self.z += self.vz * dt


class Wind:
    """Constant drift + gusts as an Ornstein-Uhlenbeck process (rms `rms`, correlation time `tau`)."""

    def __init__(self, mean, rms, tau, rng):
        self.mean, self.rms, self.tau, self.rng = mean, rms, tau, rng
        self.g = [0.0, 0.0]

    def step(self, dt):
        if self.rms > 0:
            k = math.sqrt(2 * dt / self.tau) * self.rms
            for i in range(2):
                self.g[i] += -self.g[i] / self.tau * dt + k * self.rng.gauss(0, 1)
        return (self.mean[0] + self.g[0], self.mean[1] + self.g[1])


class Person:
    """static | walk (slow circle, 0.15 m/s) | route (demo walk with stops, 0.5 m/s) | random (seeded waypoints) |
    real (the room camera's person, pushed in by World.set_real_person; unseen until the first fix).
    A person does not walk into a 1.1 m balloon: when the next step would come within KEEP m of its centre the
    step is deflected around it (people sidestep; the follower must still back off or it gets pushed)."""
    ROUTE = [((2.0, 0.0), 6.0), ((3.0, 1.2), 5.0), ((-0.5, 1.2), 6.0), ((-1.0, -1.0), 5.0), ((1.5, -1.2), 5.0)]
    KEEP = config.R_BALLOON + 0.35

    def __init__(self, mode, rng, arena=config.ARENA, speed=0.5):
        self.mode, self.rng, self.arena, self.speed = mode, rng, arena, speed
        self.t = 0.0
        self.p = [2.0, 0.0, 1.4]
        self.i, self.wait = 0, self.ROUTE[0][1] if mode == "route" else 0.0
        self.target = None
        self.blocked = 0.0
        self.detour = None
        if mode == "random":
            self.p = [1.5, 0.0, 1.4]; self.wait = 3.0
        if mode == "walk":
            self.p = [3.5, 0.0, 1.4]
        self.seen = mode != "real"          # real: False until the camera reports a person, False again when it loses them

    def _pick_random(self):
        lo, hi = self.arena
        m = 0.9
        return (self.rng.uniform(lo[0] + m, hi[0] - m), self.rng.uniform(lo[1] + m, hi[1] - m))

    def _move_toward(self, target, speed, dt, balloon):
        dx, dy = target[0] - self.p[0], target[1] - self.p[1]
        d = math.hypot(dx, dy)
        if d < 1e-9:
            return True
        step = min(d, speed * dt)
        ux, uy = dx / d, dy / d
        if balloon is not None and self.detour is None:
            bx, by = balloon[0] - self.p[0], balloon[1] - self.p[1]
            bd = math.hypot(bx, by)
            if bd < self.KEEP + 0.2 and (bx * ux + by * uy) > 0.3 * bd:   # balloon close and in the way
                self.blocked += dt
                if self.blocked < 3.0:                                   # people wait a moment for it to move off...
                    if bd < self.KEEP:                                   # (backing off if it is really on top of them)
                        self.p[0] -= bx / bd * speed * dt; self.p[1] -= by / bd * speed * dt
                    return False
                side = 1.0 if (bx * uy - by * ux) > 0 else -1.0         # ...then walk round it via a detour point
                px, py = -side * by / bd, side * bx / bd                 # perpendicular to the balloon direction
                self.detour = (balloon[0] + px * 1.4, balloon[1] + py * 1.4)
                self.blocked = 0.0
            else:
                self.blocked = max(0.0, self.blocked - dt)
        if self.detour is not None:                                      # walking round the balloon
            dx, dy = self.detour[0] - self.p[0], self.detour[1] - self.p[1]
            dd = math.hypot(dx, dy)
            if dd < 0.25:
                self.detour = None
            else:
                ux, uy = dx / dd, dy / dd; step = speed * dt
        (x0, y0), (x1, y1) = self.arena
        self.p[0] = min(x1 - 0.6, max(x0 + 0.6, self.p[0] + ux * step))    # people keep ~0.6 m off the walls
        self.p[1] = min(y1 - 0.6, max(y0 + 0.6, self.p[1] + uy * step))
        if balloon is not None:                                            # and never inside the envelope
            bx, by = self.p[0] - balloon[0], self.p[1] - balloon[1]
            bd = math.hypot(bx, by)
            if 1e-9 < bd < self.KEEP:
                self.p[0] = balloon[0] + bx / bd * self.KEEP; self.p[1] = balloon[1] + by / bd * self.KEEP
        return math.hypot(target[0] - self.p[0], target[1] - self.p[1]) < 0.05

    def step(self, dt, balloon=None):
        self.t += dt
        if self.mode in ("static", "real"):
            return
        if self.mode == "walk":
            tgt = (2.0 + 1.5 * math.cos(0.1 * self.t), 1.5 * math.sin(0.1 * self.t))
            self._move_toward(tgt, 0.35, dt, balloon)
            return
        if self.wait > 0:
            self.wait -= dt
            return
        if self.target is None:
            if self.mode == "route":
                self.i = (self.i + 1) % len(self.ROUTE); self.target = self.ROUTE[self.i][0]
            else:
                self.target = self._pick_random()
        if self._move_toward(self.target, self.speed, dt, balloon):
            self.wait = self.ROUTE[self.i][1] if self.mode == "route" else self.rng.uniform(2.0, 6.0)
            self.target = None


class World:
    """Balloon + room + person + sensors. `advance(dt)` runs the fixed-step mixer/physics; `command()` feeds it
    the laptop's JSON; `poll_state()` / `poll_telem()` hand back the datagrams that would arrive now."""

    def __init__(self, realism=REAL, person="walk", psi0=0.8, wind=(0.0, 0.0), seed=0, epoch=0.0,
                 arena=config.ARENA, obstacles=None, start=(0.0, 0.0, 1.5), person_speed=0.5, person_start=None, person_arena=None):
        self.r = dict(realism)
        self.rng = random.Random(seed)
        self.rng_tof = random.Random(seed * 7919 + 1)   # own stream: the ToF must not reshuffle gusts / dropouts / the person for seeded runs
        self.epoch = epoch                       # seconds added to message timestamps (wall clock base)
        self.t = 0.0; self.acc = 0.0
        self.arena = arena
        self.obstacles = list(config.OBSTACLES if obstacles is None else obstacles)
        self.b = Balloon(*start, psi0, self.r["motor_gain"], self.r["motor_tau"], self.r["s_yaw_arm"], self.r.get("supply_sag", 0.0))
        self.wind = Wind(wind, self.r["gust_rms"], self.r["gust_tau"], self.rng)
        self.wind_now = tuple(wind)
        self.person = Person(person, self.rng, person_arena or arena, person_speed)   # person_arena: keep the walk off the walls
        if person_start is not None: self.person.p = [person_start[0], person_start[1], 1.4]
        # "firmware" state
        self.sp, self.arm, self.rx_t = (0.0, 0.0, 0.0, 0.0), False, None
        self.mix_state = {"yawI": 0.0}
        self.cur, self.armed = (0.0, 0.0, 0.0, 0.0), False
        self.motor_override = None               # (L, R, S, V) duties pushed by the BLE bridge stand-in; None = own mixer
        self.yaw_gyro, self.gz_meas = 0.0, 0.0
        self.age_ms = -1
        # queues
        self.cmd_q, self.state_q, self.telem_q = [], [], []
        self.next_vision = self.next_telem = self.next_fpv = 0.0
        self.person_drop_until = self.balloon_drop_until = -1.0
        # stats
        self.collisions = 0; self.grazes = 0; self._touching = set(); self.collision_log = []
        self.min_clearance = 9.0; self.min_person_dist = 9.0; self.max_impact = 0.0   # m/s into a wall / obstacle, worst
        self.arm_events = 0; self.disarm_events = 0
        self.tof_valid = self.tof_total = 0
        self.fpv_seen = self.fpv_total = 0

    # ------------------------------------------------------------------ inputs
    def command(self, d):
        if self.rng.random() < self.r["cmd_loss"]:
            return
        self.cmd_q.append((self.t + self.r["cmd_latency"], d))

    # ------------------------------------------------------------------ time
    def advance(self, dt):
        self.acc += dt
        while self.acc >= MIX_DT - 1e-9:
            self.acc -= MIX_DT
            self._tick()

    def _tick(self):
        r, b = self.r, self.b
        self.t += MIX_DT
        while self.cmd_q and self.cmd_q[0][0] <= self.t:
            _, d = self.cmd_q.pop(0)
            self.sp = tuple(clamp(float(d.get(k, 0.0)), -1, 1) for k in ("vf", "vs", "yr", "vz"))
            self.arm = int(d.get("arm", 0)) == 1
            self.rx_t = self.t
        self.age_ms = int((self.t - self.rx_t) * 1000) if self.rx_t is not None else -1
        ok = self.arm and 0 <= self.age_ms < FAILSAFE_MS
        if self.motor_override is not None:                    # motors set from outside (laptop/control/ble_gondola.py --fake)
            self.cur = tuple(clamp(m, -1, 1) for m in self.motor_override)
            on = any(abs(m) > 1e-6 for m in self.cur)
            if on and not self.armed: self.arm_events += 1
            if self.armed and not on: self.disarm_events += 1
            self.armed = on
        elif ok:
            if not self.armed: self.arm_events += 1
            self.armed = True
            self.cur = mix(self.sp, self.gz_meas / YR_MAX, self.cur, self.mix_state)
        else:
            if self.armed: self.disarm_events += 1
            self.armed, self.cur = False, (0.0, 0.0, 0.0, 0.0)
            self.mix_state["yawI"] = 0.0                        # firmware resets the integrator when disarmed

        lift = r["lift_drift_n"] * math.sin(2 * math.pi * self.t / r["lift_drift_period"]) if r["lift_drift_n"] else 0.0
        n = max(1, round(MIX_DT / PHYS_DT))
        for _ in range(n):
            self.wind_now = self.wind.step(MIX_DT / n)
            b.step(self.cur, MIX_DT / n, self.wind_now, lift)
            self._collide()
        self.gz_meas = b.w + r["gyro_bias"] + (self.rng.gauss(0, r["gyro_noise"]) if r["gyro_noise"] else 0.0)
        self.yaw_gyro = wrap(self.yaw_gyro + self.gz_meas * MIX_DT)
        self.person.step(MIX_DT, (b.x, b.y))
        self.min_person_dist = min(self.min_person_dist, math.hypot(b.x - self.person.p[0], b.y - self.person.p[1]))

        if self.t >= self.next_vision:
            self.next_vision += 1.0 / r["vision_hz"]
            self._vision_frame()
        if r.get("fpv") and self.t >= self.next_fpv:
            self.next_fpv += 1.0 / r["fpv_hz"]
            self._fpv_frame()
        if self.t >= self.next_telem:
            self.next_telem += 1.0 / r["telem_hz"]
            if self.rx_t is not None and self.t - self.rx_t < 2.0:      # firmware: telemetry only while commanded
                self._telem_frame()

    # ------------------------------------------------------------------ room
    BUMP_V = 0.05        # m/s normal impact speed that counts as a bump (slower = a graze, logged separately)

    def _collide(self):
        """Walls (arena) and cylinders. Inelastic-ish: push out, kill the inward velocity (restitution 0.2).
        A contact counts as a collision only if the balloon arrives at > BUMP_V into the surface; a latex envelope
        brushing a wall at a few cm/s is a graze (counted in `grazes`)."""
        b, R = self.b, config.R_BALLOON
        (x0, y0), (x1, y1) = self.arena
        hits, grazes = set(), set()
        for key, d, nx, ny in (("xlo", b.x - x0 - R, 1, 0), ("xhi", x1 - R - b.x, -1, 0),
                               ("ylo", b.y - y0 - R, 0, 1), ("yhi", y1 - R - b.y, 0, -1)):
            self.min_clearance = min(self.min_clearance, d)
            if d < 0:
                b.x -= d * nx; b.y -= d * ny
                vn = b.vx * nx + b.vy * ny
                (hits if vn < -self.BUMP_V else grazes).add(key)
                if vn < 0: self.max_impact = max(self.max_impact, -vn); b.vx -= 1.2 * vn * nx; b.vy -= 1.2 * vn * ny
        for i, (cx, cy, cr) in enumerate(self.obstacles):
            dx, dy = b.x - cx, b.y - cy
            dist = math.hypot(dx, dy)
            d = dist - (cr + R)
            self.min_clearance = min(self.min_clearance, d)
            if d < 0:
                nx, ny = (dx / dist, dy / dist) if dist > 1e-9 else (1.0, 0.0)
                b.x -= d * nx; b.y -= d * ny
                vn = b.vx * nx + b.vy * ny
                (hits if vn < -self.BUMP_V else grazes).add(i)
                if vn < 0: self.max_impact = max(self.max_impact, -vn); b.vx -= 1.2 * vn * nx; b.vy -= 1.2 * vn * ny
        for key in (("floor", b.z - 0.6 - 0.0, 1), ("ceil", 2.8 - b.z, -1)):
            if key[1] < 0:
                hits.add(key[0]); b.z -= key[1] * key[2]
                if b.vz * key[2] < 0: b.vz = 0.0
        for h in hits - self._touching: self.collision_log.append((round(self.t, 1), h))
        self.collisions += len(hits - self._touching)
        self.grazes += len(grazes - self._touching - hits)
        self._touching = hits | grazes

    # ------------------------------------------------------------------ sensors
    def _ms(self, t):
        return int((self.epoch + t) * 1000)

    def _vision_frame(self):
        r, b, rng, t = self.r, self.b, self.rng, self.t
        if not r.get("vision", True):
            return                                              # no room camera: nothing on 5007 but the eye
        if t > self.balloon_drop_until and rng.random() < r["balloon_drop_p"]:
            self.balloon_drop_until = t + rng.uniform(*r["balloon_drop_s"])
        if t > self.person_drop_until and rng.random() < r["person_drop_p"]:
            self.person_drop_until = t + rng.uniform(*r["person_drop_s"])
        g = lambda s: rng.gauss(0, s) if s > 0 else 0.0
        balloon = None if t <= self.balloon_drop_until else [round(b.x + g(r["balloon_noise"]), 3),
                                                            round(b.y + g(r["balloon_noise"]), 3),
                                                            round(b.z + g(r["z_noise"]), 3)]
        p = self.person.p
        if t <= self.person_drop_until or not self.person.seen:
            person = None
        elif r["person_outlier_p"] and rng.random() < r["person_outlier_p"]:
            a, d = rng.uniform(0, 2 * math.pi), rng.uniform(1.0, 2.0)
            person = [round(p[0] + d * math.cos(a), 3), round(p[1] + d * math.sin(a), 3), p[2]]
        else:
            person = [round(p[0] + g(r["person_noise"]), 3), round(p[1] + g(r["person_noise"]), 3), p[2]]
        msg = {"t": self._ms(t), "balloon": balloon, "person": person, "person_id": 1, "src": "sim"}
        self.state_q.append((t + r["vision_latency"] + r["vision_jitter"] * rng.uniform(-1, 1), msg))

    def _fpv_frame(self):
        """The eye on the balloon: where the person is in the gondola camera (bearing / elevation / range), from truth.
        Camera at the balloon centre looking along the heading, tilted down fpv_pitch."""
        r, b, rng, t, p = self.r, self.b, self.rng, self.t, self.person.p
        dx, dy, dz = p[0] - b.x, p[1] - b.y, p[2] - b.z
        horiz = math.hypot(dx, dy); dist = math.sqrt(horiz * horiz + dz * dz)
        bearing = wrap(math.atan2(dy, dx) - b.psi)
        elev = math.atan2(dz, horiz) + math.radians(r["fpv_pitch"])      # measured from the (tilted) camera axis
        obs = None
        if abs(bearing) < math.radians(r["fpv_hfov"]) / 2 * 0.95 and abs(elev) < math.radians(r["fpv_vfov"]) / 2 * 0.95 \
                and not (r["fpv_drop_p"] and rng.random() < r["fpv_drop_p"]):
            g = lambda s: rng.gauss(0, s) if s > 0 else 0.0
            rng_m = None if dist < r["fpv_cut_m"] else round(dist * (1 + g(r["fpv_range_noise"])), 3)
            obs = {"bearing": round(bearing + g(r["fpv_bearing_noise"]), 4), "elev": round(elev, 4), "range": rng_m, "conf": 0.9, "box": None}
        self.fpv_total += 1; self.fpv_seen += 1 if obs else 0
        msg = {"t": self._ms(t), "balloon": None, "person": None, "fpv": obs, "src": "sim"}
        self.state_q.append((t + r["fpv_latency"], msg))

    def _telem_frame(self):
        r, c = self.r, self.cur
        if self.rng.random() < r["telem_loss"]:
            return
        msg = {"t": self._ms(self.t), "yaw": round(self.yaw_gyro, 4), "yr": round(self.gz_meas, 4), "pitch": round(self.b.tilt, 3),
               "roll": 0.0, "alt": self._tof(), "vbat": -1, "armed": int(self.armed), "age": self.age_ms,
               "mL": round(c[0], 3), "mR": round(c[1], 3), "mS": round(c[2], 3), "mV": round(c[3], 3)}
        self.telem_q.append((self.t + r["telem_latency"], msg))

    def _tof(self):
        """VL53L0X range lens -> floor, m; -1 when there is nothing (HAS_TOF 0, timeout, beyond tof_max). The lens hangs
        TOF_BELOW under the centre on the gondola pendulum: tilted, it sits a little higher and its beam reads a little long."""
        r, b = self.r, self.b
        self.tof_total += 1
        if not r.get("tof", True) or (r["tof_drop_p"] and self.rng_tof.random() < r["tof_drop_p"]):
            return -1.0
        ct = math.cos(b.tilt)
        h = (b.z - b.TOF_BELOW * ct) / ct + r["tof_bias"]
        if r["tof_false_p"] and self.rng_tof.random() < r["tof_false_p"]:
            h = self.rng_tof.uniform(0.1, 0.6 * h)                       # a hand, a cable, the person: nearer than the floor
        if r["tof_noise"]:
            h += self.rng_tof.gauss(0, r["tof_noise"])
        if not 0.03 < h < r["tof_max"]:
            return -1.0
        self.tof_valid += 1
        return round(h, 3)

    def _drain(self, q):
        due = [m for (td, m) in q if td <= self.t]
        q[:] = [(td, m) for (td, m) in q if td > self.t]
        return due

    def set_real_person(self, xyz):
        """--person real: the room camera's fix (x, y, z) in metres, or None when it does not see anyone."""
        if xyz is None:
            self.person.seen = False
        else:
            self.person.p = [float(xyz[0]), float(xyz[1]), float(xyz[2]) if len(xyz) > 2 else 1.4]
            self.person.seen = True

    def poll_state(self):
        return self._drain(self.state_q)

    def poll_telem(self):
        return self._drain(self.telem_q)

    # ------------------------------------------------------------------ truth (for tests / plots only)
    @property
    def truth(self):
        b = self.b
        return dict(x=b.x, y=b.y, z=b.z, psi=b.psi, vx=b.vx, vy=b.vy, vz=b.vz, w=b.w, person=tuple(self.person.p),
                    offset=wrap(b.psi - self.yaw_gyro), wind=self.wind_now, motors=tuple(self.cur), armed=self.armed)
