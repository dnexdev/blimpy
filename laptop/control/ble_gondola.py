"""BLE bridge: the laptop side of the gondola's Bluetooth firmware. Everything above it (teleop, follow_me, pilot, the
estimator, the telemetry watchdog, the scenario suite) keeps speaking PROTOCOL.md over UDP; this process turns that into
the firmware's per-motor percentages and turns the firmware's IMU lines back into telemetry.

    udp 5005 command {vf,vs,yr,vz,arm} --> mixer (50 Hz, yaw-rate PI on the IMU) --> "MOTORS c d e f" over BLE (20 Hz)
    BLE IMU notifications -----------> laptop/control/imu_store (latest/history/subscribe, http :5008) --> udp 5006 telemetry

  python -m laptop.control.ble_gondola                    # scan by service UUID, connect, bridge. Then run teleop / pilot as usual
  python -m laptop.control.ble_gondola --fake --sim       # no hardware: same bridge on the simulated balloon (state on 5007)
  python -m laptop.control.ble_gondola --probe            # connect and print raw IMU lines for 10 s (set config.BLE IMU_FIELDS)
  python -m laptop.control.ble_gondola --motor C 30       # bench: one motor at 30 % for 2 s (--secs 10 for a meter), then STOP (which letter is which?)

Firmware command text (write-without-response on COMMAND_UUID): "C 40" / "D -40" / "E 50" / "F 100" (one motor, percent,
sign = direction), "ALL 30", "MOTORS c d e f" (percent for C D E F), "STOP". Telemetry characteristic notifies one IMU text
line per sample. Motor letters <-> our L/R/S/V, signs and IMU layout live in config.BLE: VERIFY them on the bench.

Failsafe lives HERE now: no command for FAILSAFE_MS (500) or arm 0 or BLE lost -> STOP, mixer reset, armed 0 in telemetry.
The firmware keeps the last percentages if THIS process dies, so keep a STOP key handy and ask the hardware team for a
command timeout in the firmware too.
"""
import argparse, asyncio, json, math, queue, sys, threading, time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from .. import config
from . import imu_store
from .protocol import CMD_PORT, FAILSAFE_MS, MIX_DT, REAL_PERSON_PORT, STATE_PORT, TELEM_PORT, YR_MAX, UdpJson, clamp, mix, now_ms, wrap

B = config.BLE
LETTERS = ("C", "D", "E", "F")                 # the firmware's MOTORS argument order


def to_pct(cur):
    """Mixer duties (L, R, S, V) -> {"C": pct, ...} using config.BLE MOTORS / SIGN / PCT_MAX."""
    out = {l: 0 for l in LETTERS}
    for name, duty in zip(("L", "R", "S", "V"), cur):
        out[B["MOTORS"][name]] = int(round(clamp(duty * B["SIGN"][name] * 100.0, -B["PCT_MAX"], B["PCT_MAX"])))
    return out


def from_pct(pct):
    """Inverse of to_pct (simulator / tests): {"C": pct, ...} -> (L, R, S, V) duties."""
    return tuple(clamp(pct.get(B["MOTORS"][n], 0) / 100.0 * B["SIGN"][n], -1, 1) for n in ("L", "R", "S", "V"))


def motors_line(pct):
    return "MOTORS " + " ".join(str(pct[l]) for l in LETTERS)


# ---------------------------------------------------------------------------------------------------- transports
class Transport:
    """What the bridge needs from a link: connected flag, send(text), on_line(text) callback, connects counter."""
    connected = False
    connects = 0
    on_line = None
    def start(self): return self
    def send(self, text): raise NotImplementedError
    def close(self): pass
    def wait_closed(self, timeout=5.0): return True


def lines_of(data):
    """One BLE notification -> the IMU lines in it. Normally one; a firmware that batches sends several, newline-separated."""
    text = bytes(data).decode("utf-8", "replace") if isinstance(data, (bytes, bytearray)) else str(data)
    return [s for s in (x.strip() for x in text.replace("\r", "\n").split("\n")) if s]


def probe_verdict(got, secs):
    """The probe's last line: how many IMU lines arrived in `secs` and whether that rate is enough. The laptop prints every
    notification the moment it arrives (bleak callback, no timer), so this is the rate the firmware actually notifies at.
    Bench 2026-09-19: 10 lines in 10 s (the hardware team's firmware notified once a second while reading the IMU every 10 ms)."""
    n = len(got); ok = [t for t, d in got if d is not None]
    if n == 0: return f"[probe] 0 lines in {secs:.0f} s: nothing arrived. Is the telemetry characteristic notifying?"
    gaps = sorted(b - a for a, b in zip(ok, ok[1:]))
    hz = n / secs
    need = 1000.0 / B.get("IMU_FRESH_MS", 200)      # below this the mixer runs open loop (no yaw-rate feedback)
    s = f"[probe] {n} lines in {secs:.0f} s = {hz:.1f} per second"
    if gaps: s += f", gap median {1000 * gaps[len(gaps) // 2]:.0f} ms max {1000 * gaps[-1]:.0f} ms"
    if n - len(ok): s += f", {n - len(ok)} not parsed"
    if hz >= 20: return s + ": OK"
    if hz >= need: return s + ": SLOW (flies, but ask the hardware team for one notification per IMU sample, 20-50 per second)"
    return s + (f": TOO SLOW: the yaw loop runs blind below {need:.0f} per second. Ask the hardware team to notify every IMU sample "
                "(20-50 per second); a counter in the line (;N:123, +1 per notify) shows whether the firmware or the radio is slow")


class BleakTransport(Transport):
    """Real robot over BLE (bleak). Own asyncio loop on a thread; reconnects forever."""

    def __init__(self, name=None, log=print):
        self.name = name; self.log = log              # optional extra filter; discovery normally needs no device name
        self.loop = asyncio.new_event_loop(); self.q = None; self.client = None
        self.stopping = False; self.last_error = None; self.address = None
        self.done = threading.Event()                  # set once the link is closed for good (see wait_closed)

    def start(self):
        threading.Thread(target=self._thread, daemon=True, name="ble").start()
        return self

    def _thread(self):
        try: self.loop.run_until_complete(self._main())
        finally: self.done.set()

    def wait_closed(self, timeout=5.0):
        """After close(): block until bleak has really disconnected. Leaving the process mid-disconnect keeps Windows holding
        the link, and a box that still thinks it is connected does not advertise: the next run finds nothing."""
        return self.done.wait(timeout)

    async def _main(self):
        from bleak import BleakClient, BleakScanner
        self.q = asyncio.Queue()
        while not self.stopping:
            try:
                self.log(f"[ble] scanning for service {B['SERVICE_UUID']}"
                         + (f" with name {self.name!r}" if self.name else "") + "...")
                dev = await BleakScanner.find_device_by_filter(
                    lambda d, adv: B["SERVICE_UUID"].lower() in [u.lower() for u in (adv.service_uuids or [])]
                    and (not self.name or self.name in (d.name, adv.local_name)),
                    timeout=10.0,
                )
                if dev is None:
                    self.last_error = "not found"; self.log("[ble] service not found (powered? in range? advertising the expected service?), retrying"); await asyncio.sleep(2); continue
                self.address = dev.address
                lost = asyncio.Event()
                async with BleakClient(dev, disconnected_callback=lambda c: lost.set()) as client:
                    self.client = client
                    await client.start_notify(B["TELEMETRY_UUID"], lambda c, data: self.on_line and self.on_line(bytes(data)))
                    self.connected = True; self.connects += 1; self.last_error = None
                    self.log(f"[ble] connected to {dev.name or B['NAME']} at {dev.address}")
                    while not lost.is_set() and not self.stopping:
                        try:
                            text = await asyncio.wait_for(self.q.get(), 0.25)
                        except asyncio.TimeoutError:
                            continue
                        try:
                            await client.write_gatt_char(B["COMMAND_UUID"], text.encode(), response=False)
                        except Exception as e:
                            self.last_error = f"write: {e}"; break
                    if not lost.is_set():                      # leaving on purpose (close()): the firmware keeps the last percentages,
                        try: await client.write_gatt_char(B["COMMAND_UUID"], b"STOP", response=False)   # so the last word on the link is STOP
                        except Exception: pass
            except Exception as e:
                self.last_error = f"{e.__class__.__name__}: {e}"; self.log(f"[ble] {self.last_error}")
            finally:
                self.connected = False; self.client = None
            if not self.stopping:
                self.log("[ble] disconnected, reconnecting in 1 s"); await asyncio.sleep(1.0)

    def send(self, text):
        if not self.connected or self.q is None: return False
        self.loop.call_soon_threadsafe(self.q.put_nowait, text); return True

    def close(self):
        self.stopping = True


class SimTransport(Transport):
    """Stand-in robot: the firmware's text commands drive laptop/sim/world.py motor by motor, and its IMU lines come
    from the simulated gyro. --sim publishes the world state on 5007 like fake_esp32 does."""
    IMU_HZ = 20

    def __init__(self, world, publish_state=False, imu_units="deg"):
        self.world, self.publish_state, self.units = world, publish_state, imu_units
        self.last_cmd = None; self.last_motors = None; self.n_cmds = 0
        self.stopping = False; self._dropped = False
        self.out = UdpJson() if publish_state else None
        # --person real: the room camera's fixes (mono --port 5017) move the simulated person; nobody for 1.5 s = lost
        self.person_in = UdpJson(REAL_PERSON_PORT) if getattr(world.person, "mode", None) == "real" else None
        self.person_t = 0.0
        self.person_seen = 0

    def start(self):
        threading.Thread(target=self._run, daemon=True, name="sim-robot").start()
        return self

    def _poll_real_person(self, t):
        for m, _ in self.person_in.recv_all():
            p = m.get("person")
            if p is not None:
                self.world.set_real_person(p); self.person_t = t; self.person_seen += 1
        if self.person_t and t - self.person_t > 1.5:
            self.world.set_real_person(None); self.person_t = 0.0

    def _run(self):
        t_prev = time.monotonic(); next_imu = 0.0
        while not self.stopping:
            if self._dropped:                                 # test hook: pretend the link died for a moment
                self.connected = False; self.world.motor_override = (0.0, 0.0, 0.0, 0.0)
                time.sleep(0.5); self._dropped = False
                self.connected = True; self.connects += 1; continue
            if not self.connected:
                self.connected = True; self.connects += 1
            t = time.monotonic(); dt = min(0.25, t - t_prev); t_prev = t
            if self.person_in is not None: self._poll_real_person(t)
            self.world.advance(dt)
            if self.out is not None:
                for m in self.world.poll_state(): self.out.send(m, ("127.0.0.1", STATE_PORT))
            if t >= next_imu and self.on_line:
                next_imu = t + 1.0 / self.IMU_HZ
                k = 180.0 / math.pi if self.units == "deg" else 1.0
                w = self.world
                alt = w._tof()                                        # the sim's altimeter model (noise, dropouts); -1 = none
                scale = {"m": 1.0, "cm": 100.0, "mm": 1000.0}.get(B.get("ALT_UNITS", "m"), 1.0)
                self.on_line(f"yaw={w.yaw_gyro * k:.3f} pitch={w.b.tilt * k:.3f} roll=0.000 gx=0.000 gy=0.000 gz={w.gz_meas * k:.3f}"
                             + (f" alt={alt * scale:.1f}" if alt > 0 else " alt=-1"))
            time.sleep(0.005)

    def send(self, text):
        if not self.connected: return False
        self.n_cmds += 1; self.last_cmd = text
        parts = text.split()
        pct = dict(self.last_motors or {l: 0 for l in LETTERS})
        try:
            if parts[0] == "STOP": pct = {l: 0 for l in LETTERS}
            elif parts[0] == "ALL": pct = {l: int(parts[1]) for l in LETTERS}
            elif parts[0] == "MOTORS": pct = dict(zip(LETTERS, map(int, parts[1:5])))
            elif parts[0] in LETTERS: pct[parts[0]] = int(parts[1])
            else: return False
        except (IndexError, ValueError):
            return False
        pct = {l: int(clamp(v, -100, 100)) for l, v in pct.items()}
        self.last_motors = pct
        self.world.motor_override = from_pct(pct)
        return True

    def drop(self): self._dropped = True
    def close(self): self.stopping = True
    def wait_closed(self, timeout=5.0): time.sleep(0.2); return True


# ---------------------------------------------------------------------------------------------------- the bridge
class Bridge:
    def __init__(self, transport, http_port=None, log=print, telem_hz=20):
        self.tr, self.log = transport, log
        self.cmd_in, self.out = UdpJson(CMD_PORT), UdpJson()
        self.sp, self.arm, self.rx_t, self.sender = (0.0, 0.0, 0.0, 0.0), False, None, None
        self.cur, self.mix_state, self.armed = (0.0, 0.0, 0.0, 0.0), {"yawI": 0.0}, False
        self.yaw, self.gz, self.t_imu = 0.0, 0.0, None       # heading integrated from the IMU (or its own yaw field)
        self.pitch = self.roll = 0.0
        self.gz_bias, self._gz_win = 0.0, []                  # gyro zero (raw rad/s): learnt while disarmed and still, see _zero_gyro
        self.alt, self.t_alt = -1.0, None                         # downward ultrasonic in the IMU line (config.BLE ALT_KEYS), m
        self.last_pct, self.last_line, self.t_sent = None, None, -1e9
        self.telem_hz = telem_hz; self.n_tel = 0; self.age_ms = -1
        self.stop = threading.Event()
        self.http = None
        self._unsub = imu_store.subscribe(self._on_imu)
        transport.on_line = self._on_line
        if http_port: self._serve_http(http_port)

    # ---- IMU in
    def _on_line(self, data):
        for s in lines_of(data):
            imu_store.push(s, fields=B["IMU_FIELDS"], units=B["GYRO_UNITS"])

    def _zero_gyro(self, t, gz_raw):
        """A MEMS gyro at rest does not read zero (bench: gz -0.36 deg/s, gx -2.2, steady to 0.06): integrated, that is 20
        degrees of heading a minute. While DISARMED, when the last GYRO_ZERO_S of gz stayed within GYRO_STILL_DPS and its
        mean is a plausible offset (< GYRO_BIAS_MAX_DPS), that mean is the zero. So: hold the gondola still a few seconds
        before arming. Armed = frozen (a steady turn must not be learnt as an offset)."""
        if not B.get("GYRO_ZERO", True) or self.armed: self._gz_win.clear(); return
        k = math.pi / 180.0
        self._gz_win.append((t, gz_raw)); self._gz_win = [x for x in self._gz_win if t - x[0] <= B.get("GYRO_ZERO_S", 3.0)]
        v = [x[1] for x in self._gz_win]
        if len(v) >= 3 and t - self._gz_win[0][0] >= 0.6 * B.get("GYRO_ZERO_S", 3.0) and max(v) - min(v) < B.get("GYRO_STILL_DPS", 0.6) * k:
            m = sum(v) / len(v)
            if abs(m) < B.get("GYRO_BIAS_MAX_DPS", 5.0) * k:
                if abs(m - self.gz_bias) > 0.2 * k: self.log(f"[bridge] gyro zero: gz offset {m / k:+.2f} deg/s")
                self.gz_bias = m

    def _on_imu(self, d):
        t, sg = d["t"], B.get("GYRO_SIGN", 1)
        if "gz_rad" in d: self._zero_gyro(t, d["gz_rad"])
        gz = (d.get("gz_rad", 0.0) - self.gz_bias) * sg
        if "yaw_rad" in d: self.yaw = wrap(d["yaw_rad"] * sg)
        elif "gz_rad" in d and self.t_imu is not None: self.yaw = wrap(self.yaw + gz * min(0.2, max(0.0, t - self.t_imu)))
        self.gz = gz; self.pitch = d.get("pitch_rad", 0.0); self.roll = d.get("roll_rad", 0.0)
        self.t_imu = t
        for k in B.get("ALT_KEYS", ()):
            if k in d:
                v = float(d[k]) * {"m": 1.0, "cm": 0.01, "mm": 0.001}.get(B.get("ALT_UNITS", "m"), 1.0)
                self.alt, self.t_alt = (v if v > 0 else -1.0), t
                break

    def alt_now(self, t):
        """Telemetry alt (PROTOCOL s3): metres from the sensor to the surface below, -1 when there is no fresh echo."""
        return round(self.alt, 3) if self.t_alt is not None and (t - self.t_alt) * 1000 < B.get("ALT_FRESH_MS", 300) else -1

    def imu_fresh(self, t):
        return self.t_imu is not None and (t - self.t_imu) * 1000 < B["IMU_FRESH_MS"]

    # ---- one 50 Hz tick
    def tick(self, t):
        r = self.cmd_in.recv_latest()
        if r:
            d = r[0]; self.sender = r[1][0]
            self.sp = tuple(clamp(float(d.get(k, 0.0)), -1, 1) for k in ("vf", "vs", "yr", "vz"))
            self.arm, self.rx_t = int(d.get("arm", 0)) == 1, t
        self.age_ms = int((t - self.rx_t) * 1000) if self.rx_t is not None else -1
        ok = self.arm and 0 <= self.age_ms < FAILSAFE_MS and self.tr.connected
        if ok:
            fresh = self.imu_fresh(t)
            self.cur = mix(self.sp, (self.gz / YR_MAX) if fresh else 0.0, self.cur, self.mix_state if fresh else None)
            if not fresh: self.mix_state["yawI"] = 0.0
        else:
            self.cur, self.mix_state["yawI"] = (0.0, 0.0, 0.0, 0.0), 0.0
        if self.armed != ok:
            self.log(f"[bridge] {'ARMED' if ok else f'MOTORS OFF (arm={int(self.arm)} age={self.age_ms} ms ble={int(self.tr.connected)})'}")
        self.armed = ok
        # ---- motors out (rate-limited; STOP repeats slowly as a safety heartbeat)
        if ok:
            pct = to_pct(self.cur)
            if t - self.t_sent >= 1.0 / B["HZ"] and (pct != self.last_pct or t - self.t_sent >= 0.25):
                self._send(motors_line(pct)); self.last_pct = pct
        elif self.tr.connected and (self.last_line != "STOP" or t - self.t_sent >= 1.0):
            self._send("STOP"); self.last_pct = None
        # ---- telemetry out (PROTOCOL.md s3) to whoever commands us
        if self.sender and t >= getattr(self, "_next_tel", 0.0):
            self._next_tel = t + 1.0 / self.telem_hz
            self.out.send(self.telemetry(t), (self.sender, TELEM_PORT)); self.n_tel += 1

    def _send(self, line):
        if self.tr.send(line): self.last_line, self.t_sent = line, time.monotonic()

    def telemetry(self, t=None):
        t = time.monotonic() if t is None else t
        c = self.cur
        return {"t": now_ms(), "yaw": round(self.yaw, 4), "yr": round(self.gz, 4), "pitch": round(self.pitch, 3),
                "roll": round(self.roll, 3), "alt": self.alt_now(t), "vbat": -1, "armed": int(self.armed), "age": self.age_ms,
                "mL": round(c[0], 3), "mR": round(c[1], 3), "mS": round(c[2], 3), "mV": round(c[3], 3),
                "imu_age": -1 if self.t_imu is None else int((t - self.t_imu) * 1000), "ble": int(self.tr.connected)}

    def status(self):
        return {"connected": self.tr.connected, "connects": self.tr.connects, "armed": self.armed, "age_ms": self.age_ms,
                "alt": self.alt_now(time.monotonic()),
                "setpoint": dict(zip(("vf", "vs", "yr", "vz"), self.sp)), "motors": to_pct(self.cur), "last_line": self.last_line,
                "imu": imu_store.stats(), "error": getattr(self.tr, "last_error", None)}

    def run(self):
        self.tr.start()
        try:
            while not self.stop.is_set():
                t = time.monotonic()
                self.tick(t)
                time.sleep(max(0.0, MIX_DT - (time.monotonic() - t)))
        finally:
            for _ in range(3):
                self.tr.send("STOP"); time.sleep(0.05)
            self.tr.close(); self._unsub()
            if self.http: self.http.shutdown()
            self.tr.wait_closed(5.0)

    # ---- localhost HTTP: GET /imu  /imu/history?n=200  /status
    def _serve_http(self, port):
        bridge = self

        class H(BaseHTTPRequestHandler):
            def log_message(self, *a): pass
            def do_GET(self):
                path, _, qs = self.path.partition("?")
                q = dict(p.split("=", 1) for p in qs.split("&") if "=" in p)
                if path == "/imu": body = imu_store.latest()
                elif path == "/imu/history": body = imu_store.history(n=int(q.get("n", 200)))
                elif path == "/status": body = bridge.status()
                else: self.send_response(404); self.end_headers(); return
                data = json.dumps(body).encode()
                self.send_response(200); self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(data))); self.end_headers(); self.wfile.write(data)
        self.http = ThreadingHTTPServer(("127.0.0.1", port), H)
        threading.Thread(target=self.http.serve_forever, daemon=True, name="imu-http").start()


# ---------------------------------------------------------------------------------------------------- CLI
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--name", default=None, help="optional device-name filter in addition to the service UUID; default: service UUID only")
    ap.add_argument("--fake", action="store_true", help="simulated robot instead of Bluetooth")
    ap.add_argument("--sim", action="store_true", help="with --fake: publish the simulated balloon + person on 5007")
    ap.add_argument("--person", choices=["static", "walk", "route", "random", "real"], default="walk",
                    help="real = the room camera's person (run  python -m laptop.vision.mono --auto-calib --port 5017  as well)")
    ap.add_argument("--psi0", type=float, default=0.8); ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--ideal", action="store_true", help="with --fake: perfect sensors, no wind")
    ap.add_argument("--tof", action="store_true", help="with --fake: an ultrasonic in the IMU line (default: none, like the 2026-09-19 box)")
    ap.add_argument("--no-fpv", action="store_true", help="with --fake --sim: no eye on the balloon in the 5007 state (default: on)")
    ap.add_argument("--no-vision", action="store_true", help="with --fake --sim: no room camera (no balloon / person fixes on 5007): eye + ultrasonic only")
    ap.add_argument("--plot", action="store_true", help="with --fake: live top-down plot of the simulated world (matplotlib)")
    ap.add_argument("--probe", action="store_true", help="print raw IMU lines for 10 s, then the rate verdict, and exit")
    ap.add_argument("--motor", nargs=2, metavar=("LETTER", "PCT"), help="bench: run one motor for --secs seconds (default 2), then STOP")
    ap.add_argument("--secs", type=float, default=2.0, help="with --motor: run time in seconds; 10-15 gives time to hold a meter on the driver")
    ap.add_argument("--no-http", action="store_true"); ap.add_argument("--imu-log", action="store_true", help="append samples to data/imu.jsonl")
    args = ap.parse_args()

    if args.fake:
        from ..sim.world import IDEAL, REAL, World
        world = World(dict(IDEAL if args.ideal else REAL, tof=args.tof, fpv=not args.no_fpv, vision=not args.no_vision),
                      person=args.person, psi0=args.psi0, seed=args.seed, epoch=time.monotonic())
        tr = SimTransport(world, publish_state=args.sim, imu_units=B["GYRO_UNITS"])
    else:
        tr = BleakTransport(args.name)
    if args.imu_log: imu_store.start_log("data/imu.jsonl")

    if args.probe or args.motor:
        got = []                                                   # (arrival time, sample or None), for the rate verdict
        def on_line(data):
            for s in lines_of(data):
                d = imu_store.push(s, fields=B["IMU_FIELDS"], units=B["GYRO_UNITS"]); got.append((time.monotonic(), d))
                print(f"[imu] {s!r} -> {d}")
        tr.on_line = on_line
        tr.start(); t0 = time.monotonic()
        while not tr.connected and time.monotonic() - t0 < 30: time.sleep(0.1)
        if not tr.connected: raise SystemExit("[ble] could not connect")
        if args.motor:
            letter, pct = args.motor[0].upper(), int(args.motor[1])
            try:
                secs = max(0.2, min(30.0, args.secs))          # capped: the firmware has no timeout of its own yet
                print(f"[ble] {letter} {pct} for {secs:g} s"); t2 = time.monotonic()
                while time.monotonic() - t2 < secs:                 # resent every 200 ms: a firmware command timeout (500 ms) must not cut the test short
                    tr.send(f"{letter} {pct}"); time.sleep(0.2)
            finally:                                           # Ctrl+C included; and wait for the write: the BLE thread is a daemon,
                t1 = time.monotonic()                          # leaving at once could exit before STOP is on the air
                while not tr.send("STOP") and time.monotonic() - t1 < 15: time.sleep(0.2)   # link dropped mid-run (seen on the bench): wait for the reconnect
                time.sleep(0.5); print("[ble] STOP" if tr.connected else "[ble] STOP NOT DELIVERED: link is down, cut the motor power")
        else:
            del got[:]; t1 = time.monotonic()
            try: time.sleep(10.0)
            except KeyboardInterrupt: pass                      # Ctrl+C early: still print the verdict for what arrived
            print(probe_verdict(got, max(0.5, time.monotonic() - t1)))
        tr.close(); tr.wait_closed(5.0); return          # a clean disconnect, so the box goes back to advertising at once

    br = Bridge(tr, http_port=None if args.no_http else B["HTTP_PORT"])
    print(f"[bridge] udp {CMD_PORT} -> {'simulated robot' if args.fake else 'BLE ' + (args.name or B['NAME'])} -> telemetry on {TELEM_PORT}"
          + ("" if args.no_http else f"   http://127.0.0.1:{B['HTTP_PORT']}/imu  /status") + "   (Ctrl+C = STOP + quit)")
    th = threading.Thread(target=br.run, daemon=True); th.start()
    plot = None
    if args.plot and args.fake:
        from ..sim.plot import Plot
        plot = Plot(world, "Blimpy simulator (BLE bridge)")
    try:
        next_print = 0.0
        while th.is_alive():
            time.sleep(0.1)
            if plot: plot.update(world)
            if time.monotonic() < next_print: continue
            next_print = time.monotonic() + 0.5
            s = br.status(); m = s["motors"]
            print(f"[bridge] {'BLE ' if s['connected'] else 'no link'} {'ARMED' if s['armed'] else 'off  '} age={s['age_ms']:5d} "
                  f"C={m['C']:+4d} D={m['D']:+4d} E={m['E']:+4d} F={m['F']:+4d}  imu {s['imu']['hz']:4.1f} Hz age {s['imu']['age_ms']}  "
                  f"yaw={math.degrees(br.yaw):+4.0f}   ", end="\r", flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        br.stop.set(); th.join(timeout=2); print("\n[bridge] STOP sent, bye")


if __name__ == "__main__":
    main()
