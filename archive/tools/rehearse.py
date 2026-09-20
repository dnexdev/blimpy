"""Rehearse the whole demo WITHOUT the robot: the simulated gondola (eye on the balloon + ultrasonic, live top-down plot)
behind the BLE bridge, and the real pilot with the real voice (OMNI when YIBU_API_KEY is set, else local). One command:

  python archive/tools/rehearse.py                     # then talk: "Blimpy, follow me" / "turn left" / "stop" / "set a timer for one minute"
  python archive/tools/rehearse.py --relative          # pretend there is no room camera: eye + ultrasonic only
  python archive/tools/rehearse.py --person static     # sim person stands still (default walks around)
  python archive/tools/rehearse.py --person real       # YOU are the person: the laptop webcam + mat board track you (mono.py, started
                                               # here too), the balloon is virtual: walk the room, watch it follow on the plot
  python archive/tools/rehearse.py -- --voice local    # everything after "--" goes to the pilot (python -m laptop.control.pilot -h)

The plot window shows the balloon (blue), its heading, the person (red) and the walls. The pilot prints what it heard,
the tool call, the mode and the commands. SPACE arms, ESC quits (the pilot); this script then stops the simulator.
"""
import argparse, os, pathlib, subprocess, sys, time
ROOT = pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT)); os.chdir(ROOT)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--relative", action="store_true", help="no room camera in the sim (eye + ultrasonic only) and pilot --relative")
    ap.add_argument("--person", default="walk", choices=["static", "walk", "route", "random", "real"])
    ap.add_argument("--cam", default=None, help="with --person real: camera source for mono.py (default config.SOURCES A)")
    ap.add_argument("--cam-name", default=None, help="with --person real: calibration name for mono.py (default config.CALIB_NAMES A = env BLIMPY_CALIB_A)")
    ap.add_argument("--device", default=None, help="with --person real: detector device for mono.py (default env BLIMPY_DEVICE; a Mac wants mps)")
    ap.add_argument("--psi0", type=float, default=0.3, help="initial heading (rad); 0.3 = the person starts in the eye's view")
    ap.add_argument("--no-plot", action="store_true")
    ap.add_argument("rest", nargs=argparse.REMAINDER, help="pilot arguments after --")
    a = ap.parse_args()
    rest = [x for x in a.rest if x != "--"]

    sim = [sys.executable, "-m", "laptop.control.ble_gondola", "--fake", "--sim", "--person", a.person, "--psi0", str(a.psi0)]
    if not a.no_plot: sim.append("--plot")
    if a.relative: sim.append("--no-vision"); rest = ["--relative", *rest]
    print("[rehearse] simulator:", " ".join(sim[2:]))
    proc = subprocess.Popen(sim)
    mono = None
    if a.person == "real":
        cmd = [sys.executable, "-m", "laptop.vision.mono", "--auto-calib", "--show", "--port", "5017"]
        if a.cam is not None: cmd += ["--a", a.cam]
        if a.cam_name is not None: cmd += ["--name", a.cam_name]
        if a.device is not None: cmd += ["--device", a.device]
        print("[rehearse] room camera:", " ".join(cmd[2:]), "(board in view, hands off the laptop for 3 s)")
        mono = subprocess.Popen(cmd)
    time.sleep(2.5)
    if a.person == "real" and "--omni-cam" not in rest: rest = ["--omni-cam", "room", *rest]   # mono owns the webcam: Blimpy sees ITS picture, with everybody labelled
    try:
        sys.argv = ["pilot", *rest]
        from laptop.control.pilot import main
        main()
    finally:
        for p in (proc, mono):
            if p is None: continue
            p.terminate()
            try: p.wait(timeout=3)
            except subprocess.TimeoutExpired: p.kill()
        print("[rehearse] simulator stopped")


if __name__ == "__main__":
    main()
