"""Shared wire protocol + mixer. Mirrors firmware/src/main.cpp exactly. See PROTOCOL.md.

Four reversible motors:  L, R (rear, forward axis, spaced for yaw)   S (sideways, through the centre)   V (vertical)
Command axes: vf forward, vs sideways (+ = left), yr yaw rate (+ = CCW), vz up. All -1..1 duty fractions.
"""
import json, math, socket, time

CMD_PORT   = 5005   # laptop -> ESP32
TELEM_PORT = 5006   # ESP32 -> laptop
STATE_PORT = 5007   # vision / sim -> control (localhost)
REAL_PERSON_PORT = 5017   # mono --port 5017 -> the simulator's --person real (the camera's person, a virtual balloon)

CAP = 0.5           # max motor duty after mixing
K_YR = 1.0          # yaw-rate P gain in the mixer (balloon has ~no yaw damping; needs authority)
KI_YR = 1.0         # yaw-rate I gain (per second): cancels steady torques, e.g. the sideways motor not exactly through the centre
I_YR_MAX = 0.15     # integrator clamp (motor duty)
MIX_DT = 0.02       # mixer tick
SLEW = 0.05         # max motor change per 50 Hz tick
FAILSAFE_MS = 500
YR_MAX = 1.0        # rad/s that yr = 1.0 means


def resolve(host):
    """Resolve 'wisp-9910.local' (mDNS) or an IP once, up front. Per-packet lookups would add ~50 ms each."""
    if host.replace(".", "").isdigit():
        return host
    for attempt in range(5):
        try:
            ip = socket.gethostbyname(host)
            print(f"[net] {host} -> {ip}")
            return ip
        except OSError:
            print(f"[net] cannot resolve {host} yet (is it on the hotspot? is the hotspot on?), retry {attempt + 1}/5")
            time.sleep(1.5)
    raise SystemExit(f"[net] giving up on {host}. Check the ESP32 serial monitor for its IP and pass --esp <ip>.")


def now_ms():
    return int(time.monotonic() * 1000)


def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


def wrap(a):
    """Wrap angle to (-pi, pi]."""
    a = (a + math.pi) % (2 * math.pi) - math.pi
    return math.pi if a == -math.pi else a


def make_cmd(vf, yr, vz, arm, vs=0.0):
    return {"t": now_ms(), "vf": round(clamp(vf, -1, 1), 3), "vs": round(clamp(vs, -1, 1), 3),
            "yr": round(clamp(yr, -1, 1), 3), "vz": round(clamp(vz, -1, 1), 3), "arm": 1 if arm else 0}


def mix(sp, gz_norm, cur, state=None):
    """One 50 Hz mixer tick (same maths as the firmware).
    sp = (vf, vs, yr, vz) in [-1,1]; gz_norm = measured yaw rate / YR_MAX; cur = (mL, mR, mS, mV).
    state: mutable dict holding the yaw-rate integrator ("yawI"); None = P only.
    Returns the new (mL, mR, mS, mV)."""
    vf, vs, yr, vz = sp
    err = yr - gz_norm
    yaw_i = 0.0
    if state is not None:
        yaw_i = clamp(state.get("yawI", 0.0) + KI_YR * err * MIX_DT, -I_YR_MAX, I_YR_MAX)
        state["yawI"] = yaw_i
    diff = K_YR * err + yaw_i
    tgt = (clamp(vf - diff, -CAP, CAP), clamp(vf + diff, -CAP, CAP), clamp(vs, -CAP, CAP), clamp(vz, -CAP, CAP))
    return tuple(c + clamp(t - c, -SLEW, SLEW) for c, t in zip(cur, tgt))


class UdpJson:
    """Non-blocking UDP socket speaking one JSON object per datagram."""

    def __init__(self, bind_port=None):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        if bind_port is not None:
            self.sock.bind(("0.0.0.0", bind_port))
        self.sock.setblocking(False)

    def send(self, obj, addr):
        try:
            self.sock.sendto(json.dumps(obj, separators=(",", ":")).encode(), addr)
        except OSError:
            pass

    def recv_latest(self, only_from=None):
        """Drain the queue; return (obj, addr) of the newest valid datagram, or None.
        only_from: accept only datagrams from this IP (e.g. the ESP32 we command) so a second board or a
        simulator left running cannot poison the stream."""
        latest = None
        while True:
            try:
                data, addr = self.sock.recvfrom(2048)
            except OSError:          # BlockingIOError (queue empty) or Windows ICMP-unreachable reset
                break
            if only_from is not None and addr[0] != only_from:
                continue
            try:
                latest = (json.loads(data.decode()), addr)
            except ValueError:
                pass
        return latest

    def recv_all(self, only_from=None):
        """Drain the queue; return [(obj, addr), ...] of every valid datagram, oldest first (may be empty).
        recv_latest is what a controller wants (only the newest fix matters); this is what a recorder wants."""
        out = []
        while True:
            try:
                data, addr = self.sock.recvfrom(2048)
            except OSError:
                break
            if only_from is not None and addr[0] != only_from:
                continue
            try:
                out.append((json.loads(data.decode()), addr))
            except ValueError:
                pass
        return out
