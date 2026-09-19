"""Stand-in for the ESP32 gondola: runs the SAME mixer and failsafe as firmware/src/main.cpp, on a simulated
balloon in a simulated room (laptop/sim/world.py), with realistic sensors by default.

  python -m laptop.control.fake_esp32              protocol test: prints motor outputs, fake telemetry
  python -m laptop.control.fake_esp32 --sim        + publishes world state on 5007 so follow_me.py / pilot.py
                                                   can be run with no hardware
  python -m laptop.control.fake_esp32 --sim --plot + live top-down plot (matplotlib)
  python -m laptop.control.fake_esp32 --sim --log [NAME]   + record ground truth and everything published/received (README 1c)

Realism is ON by default: 120 ms vision latency, 2-4 cm noise, person dropouts, 2 % packet loss, gyro bias,
HVAC gusts, slowly changing lift, mismatched motors, a 2 cm ToF altimeter with dropouts. --ideal turns all of it off;
--no-tof = HAS_TOF 0 (telemetry alt -1, vision-only height).
Listens for commands on 5005; telemetry goes back to the last sender on 5006; state to 127.0.0.1:5007.
"""
import argparse, math, time
from .protocol import CMD_PORT, STATE_PORT, TELEM_PORT, UdpJson
from ..sim.world import IDEAL, REAL, World


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sim", action="store_true", help="publish the simulated balloon + person on 5007")
    ap.add_argument("--plot", action="store_true", help="live top-down plot (needs matplotlib)")
    ap.add_argument("--person", choices=["static", "walk", "route", "random"], default="walk",
                    help="static | slow circle | demo route with stops (0.5 m/s) | random waypoints")
    ap.add_argument("--ideal", action="store_true", help="perfect sensors, no wind, identical motors")
    ap.add_argument("--wind", nargs=2, type=float, default=(0.0, 0.0), metavar=("WX", "WY"), help="constant drift m/s")
    ap.add_argument("--gusts", type=float, default=None, help="gust rms m/s (default 0.06; 0 with --ideal)")
    ap.add_argument("--latency", type=float, default=None, help="vision latency s (default 0.12)")
    ap.add_argument("--gyro-bias", type=float, default=None, help="gyro bias rad/s (default 0.01)")
    ap.add_argument("--psi0", type=float, default=0.8, help="true initial heading (gyro yaw starts at 0)")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--no-tof", action="store_true", help="telemetry alt = -1 (HAS_TOF 0): vision-only height hold")
    ap.add_argument("--log", nargs="?", const="", default=None, metavar="NAME",
                    help="record ground TRUTH + published state/telemetry + received commands to data/positioning/<ts>_fake[_NAME]/")
    args = ap.parse_args()

    realism = dict(IDEAL if args.ideal else REAL)
    if args.gusts is not None: realism["gust_rms"] = args.gusts
    if args.latency is not None: realism["vision_latency"] = args.latency
    if args.gyro_bias is not None: realism["gyro_bias"] = args.gyro_bias
    realism["tof"] = not args.no_tof                 # the real gondola has the ToF; the scenario suite defaults it off
    world = World(realism, person=args.person, psi0=args.psi0, wind=tuple(args.wind), seed=args.seed, epoch=time.monotonic())

    cmd_in, out = UdpJson(CMD_PORT), UdpJson()
    from ..positioning.session import open_session
    log = open_session(args.log, tag="fake", meta={"realism": "ideal" if args.ideal else "real", "person": args.person,
                                                    "seed": args.seed, "psi0": args.psi0, "wind": list(args.wind)})
    plot = None
    if args.plot:
        from ..sim.plot import Plot
        plot = Plot(world)
    sender = None
    t_prev = time.monotonic()
    next_print = next_plot = 0.0
    print(f"[fake] listening on udp {CMD_PORT}   sim={'on' if args.sim else 'off'}   "
          f"realism={'ideal' if args.ideal else 'real'}   person={args.person}   (Ctrl+C to quit)")
    try:
        while True:
            t = time.monotonic()
            dt = min(0.25, t - t_prev); t_prev = t
            r = cmd_in.recv_latest()
            if r:
                world.command(r[0]); sender = r[1][0]; log.cmd(r[0])
            was = world.armed
            world.advance(dt)
            if world.armed != was:
                print(f"\n[fake] {'ARMED' if world.armed else f'DISARM / FAILSAFE (arm={int(world.arm)} age={world.age_ms} ms)'}")
            for m in world.poll_telem():
                if sender: out.send(m, (sender, TELEM_PORT)); log.telem(m)
            states = world.poll_state()
            for m in states:
                if args.sim: out.send(m, ("127.0.0.1", STATE_PORT)); log.state(m)
            if states:                                     # ground truth next to the noisy fix it produced (same clock as m["t"])
                log.truth(dict(world.truth, t=int((world.epoch + world.t) * 1000)))
            if t >= next_print:
                next_print = t + 0.2
                b, c, sp = world.b, world.cur, world.sp
                line = (f"[fake] arm={int(world.armed)} age={world.age_ms:5d} sp=({sp[0]:+.2f},{sp[1]:+.2f},{sp[2]:+.2f},{sp[3]:+.2f}) "
                        f"motors L={c[0]:+.2f} R={c[1]:+.2f} S={c[2]:+.2f} V={c[3]:+.2f}")
                if args.sim:
                    line += f" | pos=({b.x:+.2f},{b.y:+.2f},{b.z:.2f}) psi={math.degrees(b.psi):+4.0f}deg v={b.v:.2f}"
                print(line + "   ", end="\r", flush=True)
            if plot and t >= next_plot:
                next_plot = t + 0.1
                plot.update(world)
            time.sleep(max(0.0, 0.005 - (time.monotonic() - t)))
    except KeyboardInterrupt:
        print("\n[fake] bye")
    finally:
        log.close()


if __name__ == "__main__":
    main()
