"""The gondola's IMU stream, parsed and kept where anything on the laptop can read it.

The BLE firmware notifies one text line per sample on the telemetry characteristic. The exact layout is the hardware
team's choice, so `parse()` accepts the usual shapes:
  JSON                       {"gz": -3.1, "yaw": 12.0}
  key=value / key:value      "yaw=12.0 pitch=1.2 roll=0.3 gz=-3.1"   (any separators, any order)
  bare numbers               "0.01,-0.02,0.98,1.2,-0.4,3.1"           -> names from config.BLE["IMU_FIELDS"], or by count:
                             3 = yaw pitch roll   6 = ax ay az gx gy gz   9 = + mx my mz   7 / 10 = a leading t
Angles and rates come in the firmware's units (config.BLE["GYRO_UNITS"], "deg" by default); the store adds
`gz_rad` (rad/s) and `yaw_rad` (rad, wrapped) so the control code never sees degrees.

API (thread-safe; every call is cheap):
  push(line)                 parse + store + notify subscribers; returns the sample (None if unparsable)
  latest()                   newest sample dict or None      {"t", "age_ms", "raw", <fields>, "gz_rad", "yaw_rad"?}
  history(n=None, since=None) list of samples, oldest first
  subscribe(fn)              fn(sample) on every push (called on the pusher's thread; keep it quick)
  stats()                    {"n", "n_bad", "hz", "age_ms"}
  start_log(path)            also append every sample as one JSON line (data/imu.jsonl)
The BLE bridge (laptop/control/ble_gondola.py) also serves latest()/history() over HTTP on localhost:5008.
"""
import collections, json, math, re, threading, time

_lock = threading.Lock()
_latest = None
_hist = collections.deque(maxlen=3000)          # ~2.5 min at 20 Hz
_subs = []
_n = _n_bad = 0
_t_first = None
_log = None

_KV = re.compile(r"([A-Za-z_][A-Za-z0-9_]*)\s*[:=]\s*(-?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?)")
_NUM = re.compile(r"-?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?")
BY_COUNT = {3: ("yaw", "pitch", "roll"), 6: ("ax", "ay", "az", "gx", "gy", "gz"),
            7: ("t_ms", "ax", "ay", "az", "gx", "gy", "gz"), 9: ("ax", "ay", "az", "gx", "gy", "gz", "mx", "my", "mz"),
            10: ("t_ms", "ax", "ay", "az", "gx", "gy", "gz", "mx", "my", "mz")}


def _wrap(a):
    a = (a + math.pi) % (2 * math.pi) - math.pi
    return math.pi if a == -math.pi else a


def parse(line, fields=None, units="deg"):
    """Text (or bytes) -> dict of floats, or None. Adds gz_rad / yaw_rad / pitch_rad / roll_rad when the source fields exist."""
    if isinstance(line, (bytes, bytearray)):
        line = bytes(line).decode("utf-8", "replace")
    s = line.strip()
    if s.upper().startswith("IMU:"): s = s[4:].strip()
    if not s: return None
    d = {}
    if s.startswith("{"):
        try:
            d = {k: float(v) for k, v in json.loads(s).items() if isinstance(v, (int, float))}
        except (ValueError, AttributeError):
            d = {}
    if not d:
        kv = _KV.findall(s)
        if kv:
            d = {k: float(v) for k, v in kv}
        else:
            nums = [float(x) for x in _NUM.findall(s)]
            if not nums: return None
            names = fields or BY_COUNT.get(len(nums)) or tuple(f"v{i}" for i in range(len(nums)))
            d = {k: v for k, v in zip(names, nums)}
    k = math.pi / 180.0 if units == "deg" else 1.0
    if "gz" in d: d["gz_rad"] = d["gz"] * k
    for a in ("yaw", "pitch", "roll"):
        if a in d: d[a + "_rad"] = _wrap(d[a] * k)
    return d


def push(line, t=None, fields=None, units="deg"):
    global _latest, _n, _n_bad, _t_first
    t = time.monotonic() if t is None else t
    d = parse(line, fields, units)
    with _lock:
        if d is None:
            _n_bad += 1; return None
        d = dict(d, t=t, raw=line if isinstance(line, str) else bytes(line).decode("utf-8", "replace").strip())
        _latest = d; _hist.append(d); _n += 1
        if _t_first is None: _t_first = t
        subs = list(_subs); log = _log
    if log is not None:
        try: log.write(json.dumps({k: v for k, v in d.items() if k != "raw"}) + "\n")
        except OSError: pass
    for fn in subs:
        try: fn(d)
        except Exception as e: print(f"[imu] subscriber {fn}: {e}")
    return d


def latest():
    with _lock:
        d = _latest
    if d is None: return None
    return dict(d, age_ms=int((time.monotonic() - d["t"]) * 1000))


def history(n=None, since=None):
    with _lock:
        items = list(_hist)
    if since is not None: items = [d for d in items if d["t"] >= since]
    if n is not None: items = items[-n:]
    return items


def subscribe(fn):
    with _lock:
        _subs.append(fn)
    return lambda: _subs.remove(fn) if fn in _subs else None


def stats():
    with _lock:
        d, n, bad, t0 = _latest, _n, _n_bad, _t_first
        recent = [x["t"] for x in list(_hist)[-40:]]
    hz = (len(recent) - 1) / (recent[-1] - recent[0]) if len(recent) > 2 and recent[-1] > recent[0] else 0.0
    return {"n": n, "n_bad": bad, "hz": round(hz, 1), "age_ms": None if d is None else int((time.monotonic() - d["t"]) * 1000)}


def start_log(path):
    """Append every sample as JSON lines (gitignored data/ by default)."""
    import os
    global _log
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    with _lock:
        _log = open(path, "a", encoding="utf-8")
    return path


def reset():
    """Tests only."""
    global _latest, _n, _n_bad, _t_first
    with _lock:
        _latest = None; _hist.clear(); _subs.clear(); _n = _n_bad = 0; _t_first = None


if __name__ == "__main__":
    for s in ['IMU: yaw=12.5 pitch=-1.0 roll=0.2 gz=-3.4', '0.01,-0.02,0.98,1.2,-0.4,3.1', '{"gz": 2.0, "yaw": 90}', 'garbage']:
        print(f"{s!r:50s} -> {parse(s)}")
