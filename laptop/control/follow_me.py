"""Follow-me controller (laptop side, ~15 Hz). Holonomic: the balloon can push forward/back, sideways and up,
so it drives its VELOCITY toward what it wants instead of turning-then-pushing. Yaw is used only to face the person.
Walls and obstacles from laptop/config.py are avoided by bending the desired velocity (soft potential field).

Inputs : world state on 5007 (laptop/vision/localize.py, or fake_esp32 --sim)
         telemetry on 5006 (gyro yaw + yaw rate)
Output : commands to the ESP32 on 5005

  python -m laptop.control.follow_me                       # vs:  python -m laptop.control.fake_esp32 --sim --plot
  python -m laptop.control.follow_me                       # real gondola: python -m laptop.control.ble_gondola first (Bluetooth bridge)
  python -m laptop.control.follow_me --log [NAME]          # + record state/telemetry/commands to data/positioning/ (README 1c)

Keys: SPACE arm/disarm    n  re-learn the heading (a few short pushes)
      f follow            h  hover (hold position)      ESC quit
On arm the balloon first learns which way it is facing (a few short pushes), then follows.
Person lost > 2.5 s -> hold position.  Balloon lost > 1.5 s -> DISARM.  Same brain as pilot.py, without the voice.
Gains: laptop/config.py FOLLOW.   Don't run teleop.py at the same time (both bind 5006).
"""
import argparse, math, time
from .. import config
from .protocol import CMD_PORT, STATE_PORT, TELEM_PORT, UdpJson, clamp, make_cmd, now_ms, wrap, resolve
from .link import TelemWatchdog
from .estimator import StateEstimator
from .keys import ESC, KeyPoller

G = config.FOLLOW
A = config.AVOID


REV_EFF = config.PHYS["REV_EFF"]   # a fixed-pitch prop in reverse gives ~60 % thrust; reverse requests are scaled up to compensate


def lin(u, cap, boost=True):
    """Thrust goes with duty^2. Map a 'wanted thrust fraction' u in [-cap, cap] to the duty that produces it
    linearly, so small corrections are not wasted: lin(cap) == cap, lin(0.1*cap) gives 0.1 of the cap thrust.
    Negative u (reverse) is boosted by 1/REV_EFF so braking accelerates as hard as the request says."""
    if u < 0 and boost:
        u /= REV_EFF
    u = clamp(u, -cap, cap)
    d = math.copysign(math.sqrt(abs(u) * cap), u) if cap > 0 else 0.0
    return d if abs(d) >= G["DUTY_MIN"] else 0.0


def avoid(p, v_des, obstacles=None, arena=None, v=(0.0, 0.0), sides=None):
    """Bend a world-frame desired velocity away from walls and cylinders: the speed INTO a surface is capped by the
    braking parabola sqrt(2*A_BRAKE*d) (zero at contact), judged from where the balloon will be after the velocity
    loop's lag, and inside MARGIN a push away is added. `v` = current velocity (world). `sides` (dict, kept by the
    caller) remembers which way round each obstacle we committed to, so a goal straight behind a pillar does not
    make us dither left/right forever."""
    obstacles = config.OBSTACLES if obstacles is None else obstacles
    arena = config.ARENA if arena is None else arena
    R, M, K = config.R_BALLOON, A["MARGIN"], A["K_OBS"]
    vx, vy = v_des
    (x0, y0), (x1, y1) = arena
    surfaces = [("xlo", p[0] - x0 - R, 1.0, 0.0), ("xhi", x1 - R - p[0], -1.0, 0.0),
                ("ylo", p[1] - y0 - R, 0.0, 1.0), ("yhi", y1 - R - p[1], 0.0, -1.0)]
    for i, (cx, cy, cr) in enumerate(obstacles):
        dx, dy = p[0] - cx, p[1] - cy
        dist = math.hypot(dx, dy)
        if dist > 1e-6:
            surfaces.append((i, dist - (cr + R + 0.15), dx / dist, dy / dist))   # +0.15: sliding round a curve cuts in
    sides = {} if sides is None else sides
    allowed = []
    for key, d, nx, ny in surfaces:                # n = outward normal (away from the surface)
        d = max(0.0, d)
        closing = max(0.0, -(v[0] * nx + v[1] * ny))              # current speed toward the surface
        d_eff = max(0.0, d - 0.25 - G["V_LAG_S"] * closing)        # distance left once the loop has reacted
        allow = math.sqrt(2 * G["A_WALL"] * d_eff)
        allowed.append((nx, ny, allow, closing))
        into = vx * nx + vy * ny                   # negative = moving toward the surface
        if into < -allow:
            cut = -allow - into                    # speed we may not carry into the surface...
            vx += cut * nx; vy += cut * ny
            tx, ty = -ny, nx                       # ...is redirected along it (go around / slide along the wall)
            side = sides.get(key)
            if side is None:                       # first contact with this surface: pick the side we lean to, keep it
                side = 1.0 if vx * tx + vy * ty >= 0 else -1.0
                sides[key] = side
            tx, ty = side * tx, side * ty
            along = vx * tx + vy * ty
            add = max(0.0, min(cut, G["SLIDE_V_MAX"] - along)) * min(1.0, allow / 0.15)   # no sliding while braking hard
            vx += add * tx; vy += add * ty
        if d < M / 2:                              # a push away only when really close; the speed cap does the rest
            push = K * (1 - 2 * d / M) ** 2
            vx += push * nx; vy += push * ny
        if d > M and key in sides:                 # clear of it again: forget the commitment
            del sides[key]
    for _ in range(2):                             # final pass: one surface's slide must not feed another's approach
        for nx, ny, allow, closing in allowed:
            into = vx * nx + vy * ny
            if into < -allow:
                vx += (-allow - into) * nx; vy += (-allow - into) * ny
            if allow <= 0.0 and closing > 0.03:    # URGENT: still closing with no room left. The brake must win over
                tx, ty = -ny, nx                   # any along-the-wall demand (vector saturation would dilute it)
                along = max(-0.12, min(0.12, vx * tx + vy * ty))
                out = max(0.0, vx * nx + vy * ny) + 0.5 * closing
                vx, vy = along * tx + out * nx, along * ty + out * ny
    return vx, vy


def nearest_surface(p, obstacles=None, arena=None):
    """(clearance, nx, ny): metres between the envelope and the nearest wall / obstacle (negative = touching)
    and the outward normal pointing away from it."""
    obstacles = config.OBSTACLES if obstacles is None else obstacles
    arena = config.ARENA if arena is None else arena
    (x0, y0), (x1, y1) = arena
    R = config.R_BALLOON
    best = min((p[0] - x0 - R, 1.0, 0.0), (x1 - R - p[0], -1.0, 0.0), (p[1] - y0 - R, 0.0, 1.0), (y1 - R - p[1], 0.0, -1.0))
    for cx, cy, cr in obstacles:
        dx, dy = p[0] - cx, p[1] - cy
        dist = math.hypot(dx, dy)
        if dist > 1e-6 and dist - cr - R < best[0]:
            best = (dist - cr - R, dx / dist, dy / dist)
    return best


def clearance(p, obstacles=None, arena=None):
    return nearest_surface(p, obstacles, arena)[0]


def room_ahead(p, direction, obstacles=None, arena=None):
    """Metres the envelope can travel from p along a unit world direction before touching something (approx)."""
    obstacles = config.OBSTACLES if obstacles is None else obstacles
    arena = config.ARENA if arena is None else arena
    (x0, y0), (x1, y1) = arena
    R = config.R_BALLOON
    ux, uy = direction
    room = float("inf")
    for d, nx, ny in ((p[0] - x0 - R, 1.0, 0.0), (x1 - R - p[0], -1.0, 0.0), (p[1] - y0 - R, 0.0, 1.0), (y1 - R - p[1], 0.0, -1.0)):
        toward = -(ux * nx + uy * ny)
        if toward > 1e-6: room = min(room, max(0.0, d) / toward)
    for cx, cy, cr in obstacles:
        dx, dy = cx - p[0], cy - p[1]                      # from us to the obstacle centre
        along = dx * ux + dy * uy
        if along <= 0: continue
        off = abs(dx * uy - dy * ux)                       # lateral miss distance
        if off < cr + R:
            room = min(room, max(0.0, along - math.sqrt((cr + R) ** 2 - off ** 2)))
    return room


def nudge_ok(est, obstacles=None, arena=None):
    """The heading nudge is a blind push (heading unknown). Allow it only with room to spare."""
    return est.p is not None and clearance(est.p, obstacles, arena) > G["NUDGE_CLEAR"]


def velocity_cmd(est, v_des, obstacles=None, arena=None, sides=None):
    """World-frame desired velocity -> body-frame (vf, vs) duties. Thrust = K_V * (v_des - v), rotated into the body."""
    v_des = avoid(est.p, v_des, obstacles, arena, est.v, sides)
    ax = G["K_V"] * (v_des[0] - est.v[0])
    ay = G["K_V"] * (v_des[1] - est.v[1])
    c, s = math.cos(est.psi), math.sin(est.psi)
    uf = ax * c + ay * s                            # body +x = forward
    us = -ax * s + ay * c                           # body +y = left
    if uf < 0: uf /= REV_EFF                        # reverse is weak: ask for more so the acceleration matches
    if us < 0: us /= REV_EFF
    m = max(abs(uf) / G["VF_CAP"], abs(us) / G["VS_CAP"])
    if m > 1.0:                                     # saturate as a VECTOR: keep the direction (braking toward a wall
        uf /= m; us /= m                            # must not lose out to a sideways demand)
    return lin(uf, G["VF_CAP"], boost=False), lin(us, G["VS_CAP"], boost=False)


class AltHold:
    """Vertical P + I + D. The integrator learns the ballast trim (the balloon is never exactly neutral, and lift
    changes by grams as the room warms), so height converges instead of sitting a few cm low forever."""

    def __init__(self):
        self.i, self.t = 0.0, None

    def cmd(self, z_target, est, now=None):
        now = time.monotonic() if now is None else now
        dt = 0.0 if self.t is None else min(0.5, max(0.0, now - self.t))
        self.t = now
        z_target = clamp(z_target, A["Z_MIN"], A["Z_MAX"])
        ez = z_target - est.p[2]
        self.i = clamp(self.i + G["Z_KI"] * ez * dt, -G["Z_I_MAX"], G["Z_I_MAX"])
        u = (G["K_Z"] * ez if abs(ez) > G["Z_DEADBAND"] else 0.0) - G["K_VZ"] * est.v[2] + self.i
        return lin(u, G["VZ_CAP"])


def follow_cmd(est, person, standoff=None, v_max=None, obstacles=None, arena=None, alt=None, z_target=None, now=None,
               person_v=(0.0, 0.0), sides=None, v_extra=(0.0, 0.0)):
    """Pure controller. Face the person; hold `standoff` metres from them, moving WITH them (`person_v` feed-forward)
    and closing/opening the gap no faster than we can still brake. Returns (vf, vs, yr, vz, dbg)."""
    standoff = G["D_FOLLOW"] if standoff is None else standoff
    v_max = G["V_DES_MAX"] if v_max is None else v_max
    px, py, pz = est.p
    rx, ry = person[0] - px, person[1] - py
    dist, bearing = math.hypot(rx, ry), math.atan2(ry, rx)
    e_psi = wrap(bearing - est.psi)
    yr = clamp(G["K_PSI"] * e_psi, -G["YR_CAP"], G["YR_CAP"])
    err_d = dist - standoff                                   # + = too far, - = too close
    if dist > 1e-3 and abs(err_d) > G["D_DEADBAND"]:
        ux, uy = rx / dist, ry / dist
        sp = clamp(G["K_P"] * err_d, -v_max, v_max)           # desired speed along the line to the person
        brake = math.sqrt(2 * G["A_BRAKE"] * max(0.0, abs(err_d) - G["D_DEADBAND"] / 2))
        sp = math.copysign(min(abs(sp), brake), sp)           # ...but never faster than we can still stop from
        v_des = (ux * sp, uy * sp)
    else:
        v_des = (0.0, 0.0)                                    # inside the band: just move with the person
    v_des = (v_des[0] + person_v[0] + v_extra[0], v_des[1] + person_v[1] + v_extra[1])
    pspeed = math.hypot(*person_v)
    if pspeed > 0.15 and dist < standoff + 0.6:              # person walking at us: step out of their path, sideways
        ux_, uy_ = person_v[0] / pspeed, person_v[1] / pspeed
        along = -rx * ux_ - ry * uy_                          # > 0: we are ahead of them
        lateral = rx * uy_ - ry * ux_                         # > 0: we are on their left
        if along > 0 and abs(lateral) < 1.0:
            side = 1.0 if lateral >= 0 else -1.0
            if abs(lateral) < 0.4:                                # nearly dead ahead: step toward the OPEN side
                room_l = room_ahead(est.p, (-uy_, ux_), obstacles, arena)
                room_r = room_ahead(est.p, (uy_, -ux_), obstacles, arena)
                side = 1.0 if room_l >= room_r else -1.0
            dodge = 0.35 * (1.0 - abs(lateral))
            v_des = (v_des[0] - side * dodge * uy_, v_des[1] + side * dodge * ux_)
    mag = math.hypot(*v_des)
    if mag > G["V_ABS_MAX"]:
        v_des = (v_des[0] / mag * G["V_ABS_MAX"], v_des[1] / mag * G["V_ABS_MAX"])
    vf, vs = velocity_cmd(est, v_des, obstacles, arena, sides)
    z_target = G["Z_HOLD"] if z_target is None else z_target
    if alt is not None:
        vz = alt.cmd(z_target, est, now)
    else:
        ez = z_target - pz
        vz = lin((G["K_Z"] * ez if abs(ez) > G["Z_DEADBAND"] else 0.0) - G["K_VZ"] * est.v[2], G["VZ_CAP"])
    v_close = (est.v[0] * rx + est.v[1] * ry) / dist if dist > 1e-3 else 0.0
    return vf, vs, yr, vz, {"dist": dist, "e_psi": e_psi, "v_close": v_close}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--esp", default="127.0.0.1")
    ap.add_argument("--log", nargs="?", const="", default=None, metavar="NAME", help="record state/telemetry/commands to data/positioning/<ts>_follow[_NAME]/ (laptop/positioning)")
    args = ap.parse_args()
    args.esp = resolve(args.esp)
    from .behaviors import Behaviors          # same brain as pilot.py, minus the voice
    from ..positioning.session import open_session

    state_in, tel_in, cmd_out, keys = UdpJson(STATE_PORT), UdpJson(TELEM_PORT), UdpJson(), KeyPoller()
    log = open_session(args.log, tag="follow")   # NullLog unless --log: records what this loop consumed and sent
    est = StateEstimator(alpha=G["POS_ALPHA"], beta=G["VEL_BETA"])
    wd = TelemWatchdog()                          # telemetry silence / board-side failsafe -> disarm (laptop/control/link.py)
    beh = Behaviors(lambda s: print(f"\n[blimpy] {s}"))
    armed, t_balloon = False, 0
    print(__doc__)
    try:
        while True:
            t, now = time.monotonic(), now_ms()
            for s, _ in state_in.recv_all():                  # every row: eye rows are interleaved with the vision rows
                log.state(s)
                if s.get("balloon"):
                    est.update_balloon(s["balloon"], s.get("t", now)); t_balloon = now
                if s.get("person"):
                    beh.on_person(s["person"], s.get("t"))
                if s.get("fpv"):
                    beh.on_fpv(s["fpv"], s.get("t"))
            for m, _ in tel_in.recv_all(only_from=args.esp):          # every frame: the one that says "failsafe" must not be skipped
                est.update_telem(m.get("yaw", 0.0), m.get("yr", 0.0), m.get("alt"), m.get("t"), now=t); log.telem(m)
                if (warn := wd.telem(m, t, armed)):
                    print(f"\n[blimpy] {warn}")

            while (k := keys.poll()) is not None:
                if k == " ":
                    armed = not armed; log.event("arm", armed=armed)
                    if armed:
                        wd.arm(t); beh.on_armed(est)
                        if not est.head_ok and not nudge_ok(est, beh.obstacles, beh.arena):
                            print("\n[blimpy] not much room here to learn which way I'm facing")
                elif k == "n" and armed:
                    est.forget_heading(); beh.acq_i, beh.acq_t0, beh.acq_n = 0, None, 0
                elif k == "f" and armed:
                    beh.handle({"intent": "follow_me"}, est)
                elif k == "h" and armed:
                    beh.handle({"intent": "hover"}, est)
                elif k == ESC:
                    raise KeyboardInterrupt

            vf = vs = yr = vz = 0.0
            note = "DISARMED"
            if armed:
                reason = wd.check(armed, t)
                if reason:
                    armed, note = False, f"{reason} -> disarm"; log.event("disarm", reason=reason); print(f"\n[blimpy] {reason}: motors off")
                elif est.p is None or now - t_balloon > G["BALLOON_LOST_MS"]:
                    armed, note = False, "BALLOON LOST -> disarm"; log.event("disarm", reason="balloon lost")
                else:
                    vf, vs, yr, vz, note = beh.step(est)
            est.observe_motion(vf, vs, vz, t)
            cmd = make_cmd(vf, yr, vz, armed, vs)
            cmd_out.send(cmd, (args.esp, CMD_PORT)); log.cmd(cmd)

            pos = "(%.2f,%.2f,%.2f)" % tuple(est.p) if est.p else "none"
            psi = f"{math.degrees(est.psi):+4.0f}deg" if est.psi is not None else "n/a"
            print(f"[follow] {'ARM ' if armed else 'safe'} vf={vf:+.2f} vs={vs:+.2f} yr={yr:+.2f} vz={vz:+.2f} | balloon={pos} psi={psi} z:{est.z_src} "
                  f"| {note[:40]:40s}", end="\r", flush=True)
            time.sleep(max(0.0, 1 / G["HZ"] - (time.monotonic() - t)))
    except KeyboardInterrupt:
        pass
    finally:
        for _ in range(3):
            cmd_out.send(make_cmd(0, 0, 0, False), (args.esp, CMD_PORT)); time.sleep(0.02)
        keys.close(); log.close()
        print("\n[follow] disarmed, bye" + (f"   recorded {log.n} rows -> {log.path}" if log else ""))


if __name__ == "__main__":
    main()
