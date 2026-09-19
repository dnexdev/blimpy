"""Rehearse the whole demo WITHOUT the robot: the simulated gondola (eye on the balloon + ultrasonic, live top-down plot)
behind the BLE bridge, and the real pilot with the real voice (OMNI when YIBU_API_KEY is set, else local). One command:

  python tools/rehearse.py                     # then talk: "Blimpy, follow me" / "turn left" / "stop" / "set a timer for one minute"
  python tools/rehearse.py --relative          # pretend there is no room camera: eye + ultrasonic only
  python tools/rehearse.py --person static     # sim person stands still (default walks around)
  python tools/rehearse.py -- --voice local    # everything after "--" goes to the pilot (python -m laptop.control.pilot -h)

The plot window shows the balloon (blue), its heading, the person (red) and the walls. The pilot prints what it heard,
the tool call, the mode and the commands. SPACE arms, ESC quits (the pilot); this script then stops the simulator.
"""
import argparse, os, pathlib, subprocess, sys, time
ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT)); os.chdir(ROOT)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--relative", action="store_true", help="no room camera in the sim (eye + ultrasonic only) and pilot --relative")
    ap.add_argument("--person", default="walk", choices=["static", "walk", "route", "random"])
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
    time.sleep(2.5)
    try:
        sys.argv = ["pilot", *rest]
        from laptop.control.pilot import main
        main()
    finally:
        proc.terminate()
        try: proc.wait(timeout=3)
        except subprocess.TimeoutExpired: proc.kill()
        print("[rehearse] simulator stopped")


if __name__ == "__main__":
    main()
