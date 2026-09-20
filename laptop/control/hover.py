"""Height hold alone (laptop side, ~15 Hz): the room camera measures the balloon's height, the propeller under the
gondola holds it. Nothing else runs: no forward, no sideways, no yaw, no heading to learn, no person needed.
The balloon leaks, so how much the fan has to carry changes all the time: the trim integrator follows it, and says so
when the fan has run out of range (then it is helium or ballast, not gains).

Inputs : world state on 5007 (laptop/vision/mono.py)
         telemetry on 5006 (link watchdog only)
Output : commands to the box on 5005 (vf = vs = yr = 0, vz from AltHold)

  python -m laptop.control.hover --launch                  # ONE terminal: starts the camera and the bridge itself (their
                                                           # output goes to data/logs/hover_mono.log / hover_bridge.log)
  python -m laptop.control.hover --launch mono             # the camera only: the bridge stays up in its own terminal
or THREE terminals, all running at the same time:
  python -m laptop.vision.mono --auto-calib --show         # the camera
  python -m laptop.control.ble_gondola --no-yaw-hold       # the Bluetooth bridge   (--fake --sim: no balloon needed)
  python -m laptop.control.hover                           # hold the height it has when armed
  python -m laptop.control.hover --z 1.5 --log [NAME]      # hold 1.5 m; record to data/positioning/

Keys: SPACE arm/disarm    q / e  target up / down 0.1 m    ESC quit
Balloon lost > 1.5 s -> DISARM.  Gains: laptop/config.py HOVER.   Don't run follow_me / pilot / teleop at the same time.
"""
import argparse, os, signal, subprocess, sys, time
from .. import config
from .protocol import CMD_PORT, STATE_PORT, TELEM_PORT, UdpJson, make_cmd, now_ms, resolve
from .link import TelemWatchdog
from .altitude import LiftHold
from .keys import ESC, KeyPoller

G = config.HOVER
Z_STEP = 0.1   # m per q / e press
LAUNCH = {"mono": ["laptop.vision.mono", "--auto-calib", "--show"], "bridge": ["laptop.control.ble_gondola", "--no-yaw-hold"]}


def launch(what):
    """Start the camera and the bridge as child processes (their own consoles would fight over this one's status line)."""
    os.makedirs("data/logs", exist_ok=True)
    py, venv = sys.executable, os.path.join(".venv", "Scripts" if os.name == "nt" else "bin", "python")
    if sys.prefix == sys.base_prefix and os.path.exists(venv):        # this terminal has no venv active: the children need cv2 / bleak
        py = venv; print(f"[hover] no venv active here: starting the children with {venv}")
    procs = []
    for name, mod in LAUNCH.items():
        if name not in what: continue
        path = f"data/logs/hover_{name}.log"
        procs.append((name, subprocess.Popen([py, "-u", "-m", *mod], stdout=open(path, "w"), stderr=subprocess.STDOUT,
                                             stdin=subprocess.DEVNULL), path))
        print(f"[hover] started {name}: tail -f {path}")
    return procs


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--esp", default="127.0.0.1")
    ap.add_argument("--z", type=float, default=None, metavar="M", help="height of the balloon centre to hold (default: the height it has when armed)")
    ap.add_argument("--launch", nargs="?", const="mono,bridge", default="", metavar="WHAT", help="also start the camera (mono --auto-calib --show) and the bridge "
                    "(ble_gondola --no-yaw-hold); --launch mono = the camera only (keep ONE bridge running in its own terminal: every reconnect is a new scan)")
    ap.add_argument("--onoff", action="store_true", help="the fan is flat out (HOVER ON_DUTY) or off, nothing in between (the default while config.HOVER ONOFF "
                    "is set). The balloon must sink with the fan off. Bridge: --motor-cap 1")
    ap.add_argument("--pid", action="store_true", help="P + I + D on the fan instead of on/off")
    ap.add_argument("--auto-arm", type=float, default=None, metavar="S", help="arm by itself once the box is ready and the balloon has been in view for S seconds "
                    "(the camera window takes the keyboard focus, so SPACE may never reach this terminal). Once per run; SPACE / ESC still work")
    ap.add_argument("--log", nargs="?", const="", default=None, metavar="NAME", help="record state/telemetry/commands to data/positioning/<ts>_hover[_NAME]/ (laptop/positioning)")
    args = ap.parse_args()
    args.esp = resolve(args.esp)
    from ..positioning.session import open_session

    state_in, tel_in, cmd_out, keys = UdpJson(STATE_PORT), UdpJson(TELEM_PORT), UdpJson(), KeyPoller()
    log = open_session(args.log, tag="hover")
    lift = LiftHold(onoff=False if args.pid else True if args.onoff else None)     # the whole height loop (laptop/control/altitude.py)
    wd = TelemWatchdog()
    armed, t_balloon, z_off = False, 0, 0.0
    t_seen, auto_left = None, args.auto_arm                          # --auto-arm: since when box + balloon are both there; None = used up / off
    print(__doc__)
    procs = launch(args.launch.split(",")) if args.launch else []
    try:
        while True:
            t, now = time.monotonic(), now_ms()
            for s, _ in state_in.recv_all():
                log.state(s)
                if s.get("balloon"):
                    lift.update(s["balloon"], s.get("t", now) / 1000.0); t_balloon = now
            for m, _ in tel_in.recv_all(only_from=args.esp):          # every frame: the one that says "failsafe" must not be skipped
                log.telem(m)
                if (warn := wd.telem(m, t, armed)):
                    print(f"\n[hover] {warn}")

            pressed = []
            while (k := keys.poll()) is not None:
                pressed.append(k)
            if auto_left is not None and not armed:
                ok = lift.p is not None and now - t_balloon < 300 and wd.t_last is not None and t - wd.t_last < 1.0
                t_seen = (t if t_seen is None else t_seen) if ok else None
                if t_seen is not None and t - t_seen >= auto_left:
                    pressed.append(" "); auto_left = None; print("\n[hover] auto-arm")
            for k in pressed:
                if k == " ":
                    if not armed and lift.p is None:
                        print("\n[hover] no balloon in view yet: not arming")
                        continue
                    armed = not armed; log.event("arm", armed=armed)
                    if armed:
                        wd.arm(t); lift.arm(args.z); z_off = 0.0
                        print(f"\n[hover] holding {lift.target():.2f} m ({'fan on/off' if lift.onoff else 'P+I+D'})")
                elif k in ("q", "e"):
                    z_off += Z_STEP if k == "q" else -Z_STEP
                elif k == ESC:
                    raise KeyboardInterrupt

            vz = 0.0
            note = "DISARMED"
            for name, pr, path in procs:
                if pr.poll() is not None:
                    note = f"{name} STOPPED: see {path}"
                    if armed:
                        armed = False; log.event("disarm", reason=note); print(f"\n[hover] {note}: motors off")
            if armed:
                reason = wd.check(armed, t)
                if reason:
                    armed, note = False, f"{reason} -> disarm"; log.event("disarm", reason=reason); print(f"\n[hover] {reason}: motors off")
                elif lift.p is None or now - t_balloon > G["BALLOON_LOST_MS"]:
                    armed, note = False, "BALLOON LOST -> disarm"; log.event("disarm", reason="balloon lost"); print("\n[hover] balloon lost: motors off")
                else:
                    vz = lift.cmd(t, z_off); note = lift.note
            cmd = make_cmd(0.0, 0.0, vz, armed)
            cmd_out.send(cmd, (args.esp, CMD_PORT)); log.cmd(cmd)

            z_t = lift.target(z_off)
            z = f"{lift.p[2]:.2f}" if lift.p else "none"
            tgt = f"{z_t:.2f}" if z_t is not None else "  - "
            ez = f"{z_t - lift.p[2]:+.2f}" if lift.p and z_t is not None else "  -  "
            vzm = f"{lift.v[2]:+.2f}" if lift.p else "  -  "
            box = "none" if wd.t_last is None or t - wd.t_last > 1.0 else ("flying" if wd.board_armed else "ready")
            print(f"[hover] {'ARM ' if armed else 'safe'} box:{box} z={z} target={tgt} err={ez} vz_meas={vzm} | fan={vz:+.2f} "
                  f"| {note[:44]:44s}", end="\r", flush=True)
            time.sleep(max(0.0, 1 / G["HZ"] - (time.monotonic() - t)))
    except KeyboardInterrupt:
        pass
    finally:
        for _ in range(3):
            cmd_out.send(make_cmd(0, 0, 0, False), (args.esp, CMD_PORT)); time.sleep(0.02)
        for name, pr, _ in procs:                                     # Ctrl+C, not kill: the bridge sends STOP and waits for the
            if pr.poll() is None: pr.send_signal(signal.SIGINT)       # clean Bluetooth disconnect, or the box stays off the air
        for name, pr, _ in procs:
            try: pr.wait(timeout=8)
            except subprocess.TimeoutExpired: pr.kill()
        keys.close(); log.close()
        print("\n[hover] disarmed, bye" + (f"   recorded {log.n} rows -> {log.path}" if log else ""))


if __name__ == "__main__":
    main()
