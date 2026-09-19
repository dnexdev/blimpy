"""Replay a recorded session onto the live ports: state to 127.0.0.1:5007 (and telemetry to 5006 with --telemetry).

  python -m laptop.positioning.replay data/positioning/<session>                 # real time, once
  python -m laptop.positioning.replay data/positioning/<session> --speed 3 --loop
  python -m laptop.positioning.replay data/positioning/<session> --telemetry     # ALSO feed 5006: only when no fake_esp32 / ESP32 runs

Then run follow_me / pilot as usual: to them it looks like vision (and, with --telemetry, like the gondola).
data.t is sent as recorded (consumers only use differences), shifted by a constant per lap when looping so it never
goes backwards. Pacing is on the recorder's clock, so gaps and dropouts replay as they happened.
Sessions come from  follow_me --log / pilot --log / fake_esp32 --log / positioning.record.
"""
import argparse, time
from ..control.protocol import STATE_PORT, TELEM_PORT, UdpJson
from .sources import ReplaySource


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("session", help="data/positioning/<session> directory (or its log.jsonl)")
    ap.add_argument("--speed", type=float, default=1.0, help="1 = real time, 3 = 3x faster, 0 = dump everything at once")
    ap.add_argument("--loop", action="store_true", help="start over at the end (t keeps increasing)")
    ap.add_argument("--telemetry", action="store_true", help="also send the recorded telemetry rows to 5006")
    ap.add_argument("--to", default=f"127.0.0.1:{STATE_PORT}", metavar="HOST:PORT", help="where state goes")
    args = ap.parse_args()
    host, port = args.to.rsplit(":", 1)
    state_to, telem_to = (host, int(port)), ("127.0.0.1", TELEM_PORT)
    kinds = ("state", "telem") if args.telemetry else ("state",)
    src = ReplaySource(args.session, speed=args.speed, loop=args.loop, kinds=kinds).start()
    out = UdpJson()
    n = {"state": 0, "telem": 0}
    print(f"[replay] {len(src.rows)} rows, {src.span_ms / 1000:.1f} s recorded, speed x{args.speed}, "
          f"loop={'on' if args.loop else 'off'}, state -> {state_to}" + (f", telem -> {telem_to}" if args.telemetry else ""))
    t_print = time.monotonic()
    try:
        while not src.done:
            for kind, data in src.poll_kinds():
                out.send(data, state_to if kind == "state" else telem_to); n[kind] += 1
            if time.monotonic() - t_print > 1.0:
                t_print = time.monotonic()
                print(f"[replay] lap {src.lap}  row {src.i}/{len(src.rows)}  sent state={n['state']} telem={n['telem']}   ", end="\r", flush=True)
            time.sleep(0.002)
        print(f"\n[replay] done: sent state={n['state']} telem={n['telem']}")
    except KeyboardInterrupt:
        print(f"\n[replay] stopped: sent state={n['state']} telem={n['telem']}")


if __name__ == "__main__":
    main()
