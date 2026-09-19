"""Behavior state machine: intents in, (vf, vs, yr, vz) out, spoken lines out.
Pure logic, no sockets, no wall clock (pass `now`), so it is unit-testable at 100x real time and drives either
the simulator or the real balloon.

Modes (one at a time): HOVER, FOLLOW, WANDER, GO_TO, ROTATE, DANCE.
Concurrent: timers, pomodoro, focus guard, altitude offset. Each may speak via the `say` callback.
The eye on the balloon (on_fpv, laptop/vision/fpv.py): FOLLOW yaws on the bearing it sees and, with a room camera too,
fixes the heading estimate from it at once (no learning push). Without a room camera (est.rel, pilot --relative) the eye
plus the altimeter fly FOLLOW / ROTATE / HOVER-still; GO_TO and WANDER are refused (no x/y).
"""
import math, random, time
from .. import config
from .protocol import clamp, wrap
from .follow_me import AltHold, clearance, follow_cmd, lin, nearest_surface, room_ahead, velocity_cmd

G = config.FOLLOW
F = config.FPV
REV_EFF = config.PHYS["REV_EFF"]


class Behaviors:
    def __init__(self, say, now=time.monotonic, obstacles=None, arena=None, rng=None):
        self.say, self.now = say, now
        self.obstacles = list(config.OBSTACLES if obstacles is None else obstacles)
        self.arena = config.ARENA if arena is None else arena
        self.rng = rng or random.Random()
        self.mode, self.prev_mode = "HOVER", "HOVER"
        self.person = None; self.t_person = 0.0; self._t_person_meas = 0.0; self._person_n = 0
        self.person_v = [0.0, 0.0]                          # person's walking velocity (alpha-beta on the fixes)
        self.avoid_sides = {}                               # which way round each obstacle we committed to
        self.home = None                                    # set on arm
        self.goto_target = "me"
        self.z_offset = 0.0
        self.alt = AltHold()
        self.rotate_left = 0.0; self.rotate_dir = 1; self.rotate_prev = None; self.rotate_acc = 0.0; self.rotate_t0 = 0.0; self.rotate_tp = 0.0
        self.wander_wp = None; self.wander_t = 0.0
        self.dance_end = 0.0; self.twitch_end = -1e9; self.twitch_t0 = -1e9; self.twitch_v = (0.0, 0.0); self.v_tw = (0.0, 0.0)
        self.hold_xy = None; self.stuck_since = 0.0; self.detour = None; self.wander_stuck = 0.0
        self.acq_i, self.acq_t0, self.acq_note, self.acq_n = 0, None, "", 0
        self.acq_phase, self.acq_p0, self.acq_hist, self.acq_after = "commit", (0.0, 0.0), [], "next"
        self.timers = []                                    # (deadline, message)
        self.pomo = None                                    # dict(phase, deadline, work, brk)
        self.focus = False; self.focus_last_nag = 0.0; self.focus_absent_since = None
        self.focus_report = None; self.focus_bad_n = 0   # desk-camera reports (laptop/voice/omni_watch.py), consecutive 'not working'
        self.fpv = None; self.t_fpv = -1e9                  # the eye on the balloon: last observation (bearing / elev / range)
        self.fpv_r = None; self.fpv_rr = 0.0; self._t_fpv_r = -1e9   # range track (alpha-beta): closing speed without a world velocity
        self._t_fix = -1e9                                  # last direct heading fix from the eye + room camera
        self.search_dir = 1                                 # eye lost the person: yaw slowly toward where it last saw them

    # ------------------------------------------------------------ inputs
    def on_person(self, xyz, t_ms=None):
        """Vision gives one person fix per frame (`t_ms` = its capture time). Gate single-frame jumps (false
        detections), light smoothing, and an alpha-beta walking-velocity estimate."""
        now = self.now()
        t_meas = now if t_ms is None else t_ms / 1000.0
        xyz = [float(v) for v in xyz]
        if self.person is not None and now - self.t_person < 1.0:
            dt = max(0.03, t_meas - self._t_person_meas)
            pred = [self.person[0] + self.person_v[0] * dt, self.person[1] + self.person_v[1] * dt, self.person[2]]
            jump = math.hypot(xyz[0] - pred[0], xyz[1] - pred[1])
            if jump > 0.8:
                self._person_n += 1
                if self._person_n < 3:
                    return                                  # ignore until it persists for 3 frames
                self.person_v = [0.0, 0.0]; pred = xyz
            else:
                self._person_n = 0
            res = [x - q for x, q in zip(xyz, pred)]
            xyz = [q + 0.5 * r for q, r in zip(pred, res)]
            self.person_v = [v + 0.05 / dt * r for v, r in zip(self.person_v, res[:2])]
            sp = math.hypot(*self.person_v)
            if sp > 1.5: self.person_v = [v / sp * 1.5 for v in self.person_v]
        else:
            self.person_v = [0.0, 0.0]
        self._person_n = 0 if self._person_n >= 3 else self._person_n
        self.person, self.t_person, self._t_person_meas = xyz, now, t_meas

    def on_fpv(self, obs, t_ms=None):
        """The eye on the balloon saw the person: bearing (+ = left), elev, range (None = box cut by the frame = close).
        The range gets an alpha-beta track so the closing speed is known without any world-frame velocity."""
        if not obs or obs.get("bearing") is None: return
        now = self.now()
        self.fpv, self.t_fpv = obs, now
        if abs(obs["bearing"]) > 0.15: self.search_dir = 1 if obs["bearing"] > 0 else -1
        r = obs.get("range")
        if r is None:
            self.fpv_r, self.fpv_rr = None, 0.0
        else:
            dt = now - self._t_fpv_r
            if self.fpv_r is None or dt > 1.0 or dt <= 0:
                self.fpv_r, self.fpv_rr = float(r), 0.0
            else:
                pred = self.fpv_r + self.fpv_rr * dt
                res = float(r) - pred
                self.fpv_r = pred + 0.4 * res
                self.fpv_rr = clamp(self.fpv_rr + 0.12 / dt * res, -1.5, 1.5)
            self._t_fpv_r = now

    def fpv_ok(self, now=None):
        now = self.now() if now is None else now
        return self.fpv is not None and now - self.t_fpv < F["LOST_MS"] / 1000

    def on_focus_report(self, rep):
        """Focus watcher (OMNI model looking at the desk camera every few seconds):
        {"present": bool, "working": bool, "phone": bool, "activity": "scrolling on a phone"}."""
        self.focus_bad_n = self.focus_bad_n + 1 if not rep.get("working", True) else 0
        self.focus_report = dict(rep, t=self.now())

    def on_armed(self, est):
        if est.p is not None: self.home = (est.p[0], est.p[1])

    def handle(self, it, est):
        """Apply an intent dict from laptop/voice/intent.py. Returns the sentence to speak."""
        k = it.get("intent", "none"); reply = it.get("reply", "")
        if k == "follow_me":   self._set("FOLLOW")
        elif k == "hover":     self._set("HOVER")
        elif k in ("wander", "go_to") and getattr(est, "rel", False):
            return "I can't see the room from up here, only you. I can follow you, turn, or hold still."
        elif k == "wander":    self._set("WANDER"); self.wander_wp = None
        elif k == "go_to":
            tgt = it.get("target", "me")
            if tgt == "judges" and config.JUDGES_XY is None: return "I don't know where the judges are yet."
            if tgt == "home" and self.home is None: return "I don't have a home spot yet."
            self.goto_target = tgt; self._set("GO_TO"); self.stuck_since = self.now(); self.flips = 0; self.detour = None
        elif k == "rotate":
            deg = float(it.get("degrees", 90) or 90)
            self.rotate_left, self.rotate_dir = abs(math.radians(deg)), 1 if deg >= 0 else -1
            self.rotate_prev, self.rotate_acc, self.rotate_t0 = None, 0.0, self.now()
            self._set("ROTATE")
        elif k == "altitude":
            words = (it.get("reply", "") + " " + it.get("_text", "")).lower()
            direction = it.get("direction") or ("down" if any(w in words for w in ("down", "lower", "descend")) else "up")
            self.z_offset = clamp(self.z_offset + (0.3 if direction == "up" else -0.3), -0.8, 0.8)
        elif k == "timer":
            m = float(it.get("minutes", 5) or 5)
            self.timers.append((self.now() + m * 60, f"Time's up. Your {self._mins(m)} timer is done."))
        elif k == "pomodoro":
            w, b = float(it.get("work_min", 25) or 25), float(it.get("break_min", 5) or 5)
            self.pomo = {"phase": "work", "deadline": self.now() + w * 60, "work": w, "brk": b}
        elif k == "focus_guard":
            self.focus = bool(it.get("enabled", True)); self.focus_absent_since = None
        elif k == "mood":
            if it.get("mood", "dance") in ("dance", "happy"): self.dance_end = self.now() + 6; self._set("DANCE")
        return reply

    # ------------------------------------------------------------ 15 Hz step
    def step(self, est):
        """Returns (vf, vs, yr, vz, note). est must have .p, .psi, .yaw_gyro, .v, .speed."""
        now = self.now()
        self._tick_timers(now)
        vf = vs = yr = vz = 0.0; note = self.mode
        if est.p is None:
            return 0.0, 0.0, 0.0, 0.0, "no balloon"
        rel = getattr(est, "rel", False)                         # no room camera: x/y and heading are meaningless
        person_ok = self.person is not None and now - self.t_person < G["PERSON_LOST_MS"] / 1000
        eye = self.fpv_ok(now)
        if eye and person_ok and not rel and est.yaw_gyro is not None and now - self.t_person < 0.4 and now - self._t_fix > 0.2:
            rx, ry = self.person[0] - est.p[0], self.person[1] - est.p[1]     # the eye sees the person, the room sees both:
            if math.hypot(rx, ry) > 0.5:                                        # heading = world bearing - bearing in the image
                est.fix_heading(wrap(math.atan2(ry, rx) - self.fpv["bearing"]), k=0.3); self._t_fix = now
        confident = est.head_ok and (getattr(est, "head_confident", True) or self.acq_n >= 6)
        have_heading = est.psi is not None and confident and not rel   # until a real push has taught the heading: acquire
        est.contact = (not rel) and clearance(est.p, self.obstacles, self.arena) < 0.15   # pushing on a wall teaches nothing
        if rel and self.mode in ("GO_TO", "WANDER"): self._set("HOVER")
        z_target = G["Z_HOLD"] + self.z_offset
        # when moving somewhere other than toward the person, the person is an obstacle too (body ~0.35 m)
        obs_p = self.obstacles + ([(self.person[0], self.person[1], 0.35)] if person_ok else [])
        kw = dict(obstacles=self.obstacles, arena=self.arena, alt=self.alt, z_target=z_target, now=now, sides=self.avoid_sides)
        vcap = G["V_DES_MAX"] if getattr(est, "head_confident", True) else 0.15   # heading not confirmed yet: move gently
        kwp = dict(kw, obstacles=obs_p)
        if not person_ok: self.person_v = [0.0, 0.0]
        self.v_tw = self._twitch(est, now)
        kw["v_extra"] = kwp["v_extra"] = self.v_tw
        hold_z = True
        if est.psi is not None and not confident and not rel:
            vf, vs = self._acquire(est, now); note = f"{self.mode} (learning heading: {self.acq_note})"
            vz = self.alt.cmd(z_target, est, now)
            self._focus_guard(now, person_ok)
            return vf, vs, 0.0, vz, note                        # no rotating / travelling until we know which way is which

        if self.mode == "FOLLOW":
            if eye:
                vf, yr, note = self._follow_eye(now); self.hold_xy = None        # the eye owns the yaw (seen directly)
                if person_ok and have_heading:                                    # room camera too: the world-frame law moves us
                    pv = self.person_v if math.hypot(*self.person_v) > 0.1 else (0.0, 0.0)   # (walls, dodging, the person's speed)
                    vf, vs, _, vz, d = follow_cmd(est, self.person, person_v=pv, v_max=vcap, **kw); hold_z = False
                    note = f"FOLLOW eye+room d={d['dist']:.2f}"
            elif person_ok and have_heading:
                pv = self.person_v if math.hypot(*self.person_v) > 0.1 else (0.0, 0.0)   # standing: no feed-forward noise
                vf, vs, yr, vz, d = follow_cmd(est, self.person, person_v=pv, v_max=vcap, **kw); hold_z = False; self.hold_xy = None
                note = f"FOLLOW d={d['dist']:.2f}"
            elif rel:
                yr = F["SEARCH_YR"] * self.search_dir; note = "FOLLOW (eye lost the person: searching)"
            elif have_heading:
                vf, vs = self._hold(est); note = "FOLLOW (person lost -> hold)"
        elif self.mode == "GO_TO":
            xy = self._goto_xy()
            if xy is None or not have_heading:
                note = "GO_TO (no target)"
            else:
                so = G["STANDOFF_ME"] if self.goto_target == "me" else G["STANDOFF_PLACE"]
                goal = xy
                if self.detour is not None:                     # wedged earlier: first reach the detour point
                    goal = self.detour
                    if math.hypot(goal[0] - est.p[0], goal[1] - est.p[1]) < 0.6: self.detour, goal = None, xy
                vf, vs, yr, vz, d = follow_cmd(est, [goal[0], goal[1], 0.0], standoff=so if goal is xy else 0.3, v_max=vcap,
                                               **(kw if self.goto_target == "me" else kwp)); hold_z = False
                if self.goto_target != "me": yr = self._yaw_to_travel(est, goal, d["dist"])   # nose toward the trip (strong motors)
                dist_goal = math.hypot(xy[0] - est.p[0], xy[1] - est.p[1])
                note = f"GO_TO {self.goto_target} d={dist_goal:.2f}" + (" via detour" if goal is not xy else "")
                if est.speed > 0.1: self.stuck_since = now
                if dist_goal < so + G["ARRIVE_M"] and est.speed < 0.12: self._set("HOVER"); self.say("I'm here.")
                elif now - self.stuck_since > 3.0 and dist_goal > so + 1.0:
                    self.flips = getattr(self, "flips", 0) + 1
                    self.detour = self._detour_point(est, xy, prefer=-1.0 if self.flips % 2 == 0 else 1.0)
                    self.stuck_since = now; self.avoid_sides.clear()
                    if self.flips > 3: self._set("HOVER"); self.say("I can't find a way through."); self.flips = 0
                elif now - self.stuck_since > 4.0: self._set("HOVER"); self.say("This is as close as I can get.")
        elif self.mode == "WANDER":
            if self.wander_wp is None or now > self.wander_t:
                self.wander_wp = self._wander_waypoint(); self.wander_t = now + 25
            if have_heading:
                vf, vs, yr, vz, d = follow_cmd(est, [*self.wander_wp, 0.0], standoff=0.2, v_max=min(0.25, vcap), **kwp); hold_z = False
                yr = self._yaw_to_travel(est, self.wander_wp, d["dist"])
                note = f"WANDER d={d['dist']:.2f}"
                if est.speed > 0.1 or d["dist"] < 0.8: self.wander_stuck = now
                if d["dist"] < 0.5 or now - self.wander_stuck > 4.0: self.wander_wp = None; self.avoid_sides.clear()
        elif self.mode == "ROTATE":
            if est.yaw_gyro is not None:
                if self.rotate_prev is None: self.rotate_prev, self.rotate_tp = est.yaw_gyro, now
                dyaw = wrap(est.yaw_gyro - self.rotate_prev) - est.bias_hat * (now - self.rotate_tp)
                self.rotate_acc += self.rotate_dir * dyaw; self.rotate_prev, self.rotate_tp = est.yaw_gyro, now
                rem = self.rotate_left - self.rotate_acc
                if (abs(rem) < G["ROT_DONE_RAD"] and abs(est.yr_meas) < 0.2) or now - self.rotate_t0 > 20:
                    self._set("HOVER")
                else:
                    mag = clamp(G["K_ROT"] * abs(rem), G["ROT_MIN"], G["ROT_CAP"])
                    yr = self.rotate_dir * math.copysign(mag, rem)
            if have_heading: vf, vs = self._hold(est)          # spin on the spot, don't drift
            note = f"ROTATE {math.degrees(self.rotate_acc):.0f}/{math.degrees(self.rotate_left):.0f}"
        elif self.mode == "DANCE":
            ph = self.dance_end - now
            yr = 0.5 * (1 if int(ph * 1.5) % 2 == 0 else -1)
            vs = 0.25 * (1 if int(ph * 1.0) % 2 == 0 else -1)
            vz = 0.25 * math.sin(ph * 4); hold_z = False
            if now >= self.dance_end: self._set(self.prev_mode if self.prev_mode != "DANCE" else "HOVER")
            note = "DANCE"
        elif self.mode == "HOVER":
            if have_heading: vf, vs = self._hold(est)          # hover = actively kill drift

        if hold_z:
            vz = self.alt.cmd(z_target, est, now)
        if have_heading: yr += est.bias_hat                  # hold a TRUE zero yaw rate despite gyro bias (YR_MAX = 1 rad/s)
        if self.v_tw != (0.0, 0.0): note += " +twitch"
        self._focus_guard(now, person_ok)
        return vf, vs, yr, vz, note

    ACQ_DIRS = ((0.3, 0.0), (-0.39, 0.0), (0.0, 0.42), (0.0, -0.5))   # body pushes of ~equal acceleration

    def _acquire(self, est, now):
        """Heading unknown (first arm / after `forget_heading`): push along one body axis until the learner has it.
        PROBE ~2.5 s, then look at the room ahead along the direction we actually started moving: plenty -> COMMIT
        (keep pushing), little -> the opposite axis is the safe one. Every push ends with a BRAKE on the opposite of
        the axis that produced the motion (correct whatever the estimate is; a keel-less balloon must never coast).
        Converged = the estimate has held still for 2.5 s while lessons kept coming. NEVER brake with the velocity
        loop while the heading is unknown (that can be full throttle into a wall). Gives up after 6 pushes."""
        gap, nx, ny = nearest_surface(est.p, self.obstacles, self.arena)
        lessons = getattr(est, "lessons_total", 0)
        self.acq_hist.append((now, est.offset, lessons, est.speed))
        while self.acq_hist and now - self.acq_hist[0][0] > 3.0:
            self.acq_hist.pop(0)
        old = next((h for h in self.acq_hist if now - h[0] >= 2.5), None)
        converged = est.head_ok and old is not None and abs(wrap(est.offset - old[1])) < 0.1 and lessons - old[2] >= 3
        if self.acq_t0 is None:
            self.acq_i = self._acq_pick(est, nx, ny, None)
            self._acq_start(est, now, "probe" if gap < 1.5 else "commit")
        age = now - self.acq_t0
        moved = math.hypot(est.p[0] - self.acq_p0[0], est.p[1] - self.acq_p0[1])
        if self.acq_phase == "brake":
            if est.speed < 0.08 or age > 3.5:                   # stopped (or as stopped as it gets)
                if converged or self.acq_after == "done":
                    est.head_confident = True; self.acq_note = "converged"; return (0.0, 0.0)
                if self.acq_after == "reverse":
                    self.acq_i ^= 1; self._acq_start(est, now, "commit")
                else:
                    self.acq_i = self._acq_pick(est, nx, ny, self.acq_i); self._acq_start(est, now, "probe" if gap < 1.5 else "commit")
                self.acq_n += 1; self.acq_note = "next push"
                return (0.0, 0.0)
            self.acq_note = f"brake axis {self.acq_i % 4}"
            vf, vs = self.ACQ_DIRS[self.acq_i % 4 ^ 1]
            return (0.85 * vf, 0.85 * vs)
        if converged:                                           # good estimate: stop pushing, kill the speed, done
            self._acq_brake(now, "done"); return (0.0, 0.0)
        sp = est.speed
        room = room_ahead(est.p, (est.v[0] / sp, est.v[1] / sp), self.obstacles, self.arena) if sp > 0.12 else 9.0
        old_sp = next((h[3] for h in self.acq_hist if now - h[0] >= 0.6), None)
        if room < 0.7 and age > 0.8 and old_sp is not None and est.speed > old_sp + 0.01:
            # heading into something AND this push is what speeds us up (needs no heading estimate)
            self._acq_brake(now, "reverse"); self.acq_note = "room ahead low -> brake"
            return (0.0, 0.0)
        if self.acq_phase == "probe" and age >= 2.5:            # the velocity estimate lags ~1 s: give it time to show
            if room >= 1.0:                                     # heading somewhere open: keep pushing
                self.acq_phase = "commit"; self.acq_note = "probe -> commit"
            else:                                               # heading into something: brake, then the opposite axis
                self._acq_brake(now, "reverse"); self.acq_note = "probe -> reverse"
            return (0.0, 0.0)
        if gap < 0.05 and age > 1.5:                            # pressing on something: brake, then reverse
            self._acq_brake(now, "reverse"); self.acq_note = "touching"
            return (0.0, 0.0)
        if est.speed > 0.25 or moved > 0.6 or age > G["NUDGE_S"] + 2.0:   # pushed enough: brake, then next push
            self._acq_brake(now, "next"); return (0.0, 0.0)
        self.acq_note = f"{self.acq_phase} axis {self.acq_i % 4}"
        vf, vs = self.ACQ_DIRS[self.acq_i % 4]
        k = 1.0 if gap > 1.0 else 0.85                          # something close: push a little more gently
        return (k * vf, k * vs)

    def _acq_start(self, est, now, phase):
        self.acq_t0, self.acq_phase = now, phase
        self.acq_p0 = (est.p[0], est.p[1])

    def _acq_brake(self, now, after):
        self.acq_t0, self.acq_phase, self.acq_after = now, "brake", after

    def _detour_point(self, est, xy, prefer=1.0):
        """Wedged on the way to xy: pick a point to the side of the goal line FROM WHICH the goal is reachable in a
        straight line (most room from the candidate toward the goal). `prefer` breaks ties toward one side so
        repeated attempts alternate sides."""
        bx, by = xy[0] - est.p[0], xy[1] - est.p[1]
        d = math.hypot(bx, by) or 1.0
        ux, uy = bx / d, by / d
        (x0, y0), (x1, y1) = self.arena
        m = config.R_BALLOON + 0.3
        best = None
        for side in (prefer, -prefer):
            for step in (1.0, 1.6):
                cx = min(x1 - m, max(x0 + m, est.p[0] - side * uy * step))
                cy = min(y1 - m, max(y0 + m, est.p[1] + side * ux * step))
                if clearance((cx, cy), self.obstacles, self.arena) < 0.25:
                    continue
                gx, gy = xy[0] - cx, xy[1] - cy
                gd = math.hypot(gx, gy) or 1.0
                score = min(room_ahead((cx, cy), (gx / gd, gy / gd), self.obstacles, self.arena), gd) - 0.2 * step + (0.05 if side == prefer else 0.0)
                if best is None or score > best[0]: best = (score, cx, cy)
        if best is None:
            return (min(x1 - m, max(x0 + m, est.p[0] - prefer * uy)), min(y1 - m, max(y0 + m, est.p[1] + prefer * ux)))
        return (best[1], best[2])

    def _yaw_to_travel(self, est, xy, dist):
        """Far from a place: turn the nose (the two strong motors) toward it; close by, stop turning."""
        if dist < 1.0:
            return 0.0
        bearing = math.atan2(xy[1] - est.p[1], xy[0] - est.p[0])
        return clamp(G["K_PSI"] * wrap(bearing - est.psi), -0.3, 0.3)

    def _acq_pick(self, est, nx, ny, avoid_i):
        """Next body axis to push along. With a rough estimate, prefer the one pointing away from the nearest
        surface; with none, cycle. Never repeat the axis that just failed."""
        order = [0, 1, 2, 3] if avoid_i is None else [(avoid_i + k) % 4 for k in (1, 2, 3)]
        if est.head_ok and est.psi is not None:                 # rough estimate: prefer the axis with the most room ahead
            c, s_ = math.cos(est.psi), math.sin(est.psi)
            def room(i):
                bx, by = (1.0, 0.0) if i in (0, 1) else (0.0, 1.0)
                if i in (1, 3): bx, by = -bx, -by
                return room_ahead(est.p, (bx * c - by * s_, bx * s_ + by * c), self.obstacles, self.arena)
            order.sort(key=room, reverse=True)
        return order[0]

    def _twitch(self, est, now):
        """Heading is learned from how the balloon answers a push. After a long time without a real push the gyro has
        drifted and nothing re-teaches it, so add a 2 s, 0.35 m/s velocity offset in the strongest direction that has
        room (forward beats sideways beats reverse). It goes through the normal velocity loop and avoidance, so it
        can never push into a wall, and the hold / follow controller brings the balloon back afterwards."""
        if est.psi is None or not est.head_ok or self.mode in ("ROTATE", "DANCE"):
            return (0.0, 0.0)
        if now < self.twitch_end:
            if now > self.twitch_end - 0.2 and getattr(est, "push_strength", 1.0) < 0.22 and now - self.twitch_t0 < 3.5:
                self.twitch_end = now + 0.4                     # not enough signal yet: keep pushing a little longer
            return self.twitch_v
        fresh_limit = G["TWITCH_AFTER_S"] if getattr(est, "bias_ok", False) else 0.4 * G["TWITCH_AFTER_S"]
        if est.speed > 0.2: fresh_limit *= 2                    # a lesson taken while cruising is wind-biased: wait if we can
        if getattr(est, "heading_fresh_s", 0) <= fresh_limit or now - self.twitch_end < 5:
            return (0.0, 0.0)
        c, s_ = math.cos(est.psi), math.sin(est.psi)
        obs = self.obstacles + ([(self.person[0], self.person[1], 0.35)] if self.person is not None else [])
        best = None
        for bx, by, dur in ((1.0, 0.0, 1.0), (0.0, 1.0, 1.0), (-1.0, 0.0, 1.0), (0.0, -1.0, 1.2)):
            wx, wy = bx * c - by * s_, bx * s_ + by * c
            room = room_ahead(est.p, (wx, wy), obs, self.arena)
            if best is None or room > best[0]: best = (room, wx, wy, dur)
            if room >= 1.8: break
        room, wx, wy, dur = best
        if room < 0.9:
            self.twitch_end = now                               # nowhere to push right now: retry in 5 s
            return (0.0, 0.0)
        sp = 0.35 * min(1.0, (room - 0.6) / 1.2)                # less room -> gentler push (stopping needs ~1 m at 0.35 m/s)
        self.twitch_v = (sp * wx, sp * wy)
        self.twitch_t0, self.twitch_end = now, now + G["TWITCH_S"] * dur
        return self.twitch_v

    # ------------------------------------------------------------ helpers
    def _follow_eye(self, now):
        """FOLLOW on the eye alone: yaw the bearing to zero; forward/back on the range, braking on the tracked closing
        speed. No walls, no sideways: that is what the room camera adds when it is there."""
        o = self.fpv
        yr = clamp(F["K_PSI"] * o["bearing"], -G["YR_CAP"], G["YR_CAP"])
        r = self.fpv_r if self.fpv_r is not None else 0.9            # box cut by the frame = closer than we can measure
        err = r - G["D_FOLLOW"]                                       # + = too far
        sp = 0.0
        if abs(err) > G["D_DEADBAND"]:
            sp = clamp(G["K_P"] * err, -G["V_DES_MAX"], G["V_DES_MAX"])
            brake = math.sqrt(2 * G["A_BRAKE"] * max(0.0, abs(err) - G["D_DEADBAND"] / 2))
            sp = math.copysign(min(abs(sp), brake), sp)
        u = G["K_V"] * (sp + self.fpv_rr)                              # closing speed = -d(range)/dt
        if u < 0: u /= REV_EFF
        vf = lin(u, G["VF_CAP"], boost=False)
        if abs(o["bearing"]) > 0.6: vf = min(vf, 0.0)                 # facing away: turn first, only back off meanwhile
        return vf, yr, f"FOLLOW eye b={math.degrees(o['bearing']):+.0f} r={'%.2f' % r if self.fpv_r is not None else 'close'}"

    def _hold(self, est):
        """Hold position: the spot where we entered HOVER / lost the person / started rotating. A pure velocity hold
        would integrate every gust and every heading twitch into a slow walk across the room."""
        if self.hold_xy is None:
            self.hold_xy = (est.p[0], est.p[1])
        ex, ey = self.hold_xy[0] - est.p[0], self.hold_xy[1] - est.p[1]
        d = math.hypot(ex, ey)
        if d < G["HOLD_DEADBAND"]:
            v_des = (0.0, 0.0)
        else:
            sp = min(G["K_P"] * d, math.sqrt(2 * G["A_BRAKE"] * max(0.0, d - G["HOLD_DEADBAND"] / 2)), G["HOLD_V_MAX"])
            v_des = (ex / d * sp, ey / d * sp)
        v_des = (v_des[0] + self.v_tw[0], v_des[1] + self.v_tw[1])
        return velocity_cmd(est, v_des, self.obstacles, self.arena, self.avoid_sides)

    def _set(self, mode):
        if mode != self.mode:
            self.prev_mode, self.mode = self.mode, mode
            self.hold_xy = None                                 # re-anchor the hold where the new mode starts

    def _goto_xy(self):
        t = self.goto_target
        if t == "judges": return config.JUDGES_XY
        if t == "home": return self.home
        return (self.person[0], self.person[1]) if self.person else None

    def _wander_waypoint(self):
        (x0, y0), (x1, y1) = config.WANDER_BOX or self.arena   # venues/*.json may pin a wander box; else the whole arena
        m = config.R_BALLOON + 0.35                  # sampling margin; avoid() keeps the real clearance
        for _ in range(50):
            x, y = self.rng.uniform(x0 + m, x1 - m), self.rng.uniform(y0 + m, y1 - m)
            if all(math.hypot(x - cx, y - cy) > cr + m for cx, cy, cr in self.obstacles) and \
               (self.person is None or math.hypot(x - self.person[0], y - self.person[1]) > 1.2):
                return (x, y)
        return ((x0 + x1) / 2, (y0 + y1) / 2)

    @staticmethod
    def _mins(m):
        return f"{int(m * 60)} second" if m < 1 else f"{m:g} minute"

    def _tick_timers(self, now):
        for d, msg in [t for t in self.timers if t[0] <= now]:
            self.say(msg); self.timers.remove((d, msg))
        p = self.pomo
        if p and now >= p["deadline"]:
            if p["phase"] == "work":
                p.update(phase="break", deadline=now + p["brk"] * 60); self.say(f"Work block done. Take a {p['brk']:g} minute break.")
            else:
                p.update(phase="work", deadline=now + p["work"] * 60); self.say("Break's over. Back to it.")

    def _focus_guard(self, now, person_ok):
        if not self.focus: return
        rep = self.focus_report
        if rep and now - rep["t"] < 30 and self.focus_bad_n >= 2 and now - self.focus_last_nag > 30:
            self.focus_last_nag = now                      # the camera saw two looks in a row of not working
            if not rep.get("present", True): self.say("Hey. Where did you go? Get back to work.")
            else: self.say(f"Hey. I can see you're {rep.get('activity') or ('on your phone' if rep.get('phone') else 'not working')}. Back to work.")
            return
        if person_ok:
            self.focus_absent_since = None; return
        if self.focus_absent_since is None: self.focus_absent_since = now
        elif now - self.focus_absent_since > 20 and now - self.focus_last_nag > 30:
            self.focus_last_nag = now; self.say("Hey. Where did you go? Get back to work.")
