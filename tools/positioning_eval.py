"""Score a recorded positioning session (laptop/positioning/session.py) against sim truth, another session's truth,
or a tape measure; optional PNG. Logic: laptop/positioning/evaluate.py.

  python tools/positioning_eval.py data/positioning/<ts>_fake_x --plot                 # sim truth in the same session
  python tools/positioning_eval.py <ts>_follow_x --truth-session <ts>_fake_x           # recorded together: joined on wall time
  python tools/positioning_eval.py <ts>_rec_x --expect balloon 1.20 0.40 1.70          # static balloon at a tape-measured spot
  python tools/positioning_eval.py <ts>_rec_x --expect person 2.0 0.0 1.2 --tol 0.15

Bare names resolve under config.POSITIONING_DIR. Exit 0 when every judged check passes (a session without truth judges
only --expect). PNG goes to sim_out/positioning_<session>.png.
"""
import argparse, pathlib, sys
ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from laptop import config
from laptop.positioning import evaluate, session


def resolve(name):
    p = pathlib.Path(name)
    return p if p.exists() else ROOT / config.POSITIONING_DIR / name


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("session"); ap.add_argument("--truth-session", default=None, metavar="DIR")
    ap.add_argument("--plot", action="store_true")
    ap.add_argument("--expect", nargs=4, action="append", default=[], metavar=("OBJ", "X", "Y", "Z"), help="balloon|person x y z (m)")
    ap.add_argument("--tol", type=float, default=0.10, help="m; --expect passes within this distance")
    ap.add_argument("--jump", type=float, default=0.5, help="m; a fix-to-fix move above this counts as a jump")
    args = ap.parse_args()
    sdir = resolve(args.session)
    if not (sdir / session.LOG_NAME).exists():
        sys.exit(f"no {session.LOG_NAME} in {sdir}")
    d = session.load(sdir)
    truth = None
    if args.truth_session:
        truth = evaluate.truth_from(d, session.load(resolve(args.truth_session)))
    expect = [(o, float(x), float(y), float(z)) for o, x, y, z in args.expect]
    try:
        rep = evaluate.evaluate(d, truth=truth, expect=expect, jump_m=args.jump, tol_m=args.tol)
    except ValueError as e:
        sys.exit(f"[eval] {e}")
    print(evaluate.format_report(rep, sdir.name))
    if args.plot:
        out = ROOT / "sim_out"; out.mkdir(exist_ok=True)
        print("[eval] plot:", evaluate.save_plot(d, rep, out / f"positioning_{sdir.name}.png", truth=truth))
    sys.exit(0 if evaluate.passed(rep) else 1)


if __name__ == "__main__":
    main()
