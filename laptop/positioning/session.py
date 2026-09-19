"""Record and load positioning sessions: data/positioning/<YYYYmmdd_HHMMSS>_<tag>[_NAME]/{session.json, log.jsonl}.

  python -m laptop.control.follow_me --log [NAME]           # the controller records what it consumed + what it sent
  python -m laptop.control.fake_esp32 --sim --log [NAME]    # the sim records ground truth + what it published / received
  python -m laptop.positioning.session data/positioning/<session>     # summary: rows per kind, duration, rates

  from laptop.positioning.session import load
  d = load("data/positioning/20260913_143000_follow_demo")
  d["state"]["balloon"]        # (N, 3) float, NaN rows where the balloon was not seen;  d["state"]["t"] recorder ms
  d["cmd"]["vf"], d["truth"]["x"], d["meta"]["git"] ...

Why the consumer records (and not a separate tap): protocol.UdpJson binds with SO_REUSEADDR, so a second socket on
5007 would STEAL datagrams from follow_me / pilot instead of copying them. laptop/positioning/record.py exists for
vision-only runs where no controller is listening.
Rows are appended line-buffered, so Ctrl+C, a kill, or a crash loses at most the line being written.
Row format: laptop/positioning/schema.py.
"""
import json, os, platform, subprocess, sys, time
from pathlib import Path
from .. import config
from ..control.protocol import CMD_PORT, STATE_PORT, TELEM_PORT, now_ms
from .schema import KINDS, SCHEMA_VERSION, row

LOG_NAME, META_NAME = "log.jsonl", "session.json"


def _git_hash():
    try:
        return subprocess.run(["git", "rev-parse", "--short", "HEAD"], capture_output=True, text=True, timeout=2,
                              cwd=Path(__file__).resolve().parents[2]).stdout.strip() or None
    except (OSError, subprocess.SubprocessError):
        return None


def _config_snapshot():
    """What the controller believed about the room and its gains when this was recorded."""
    return {"VENUE_FILE": str(config.VENUE_FILE), "VENUE": config.VENUE.name, "ARENA": config.ARENA,
            "OBSTACLES": config.OBSTACLES, "JUDGES_XY": config.JUDGES_XY, "WANDER_BOX": config.WANDER_BOX,
            "R_BALLOON": config.R_BALLOON, "PHYS": config.PHYS, "AVOID": config.AVOID, "FOLLOW": config.FOLLOW}


class SessionLog:
    """Append-only JSONL writer for one run. Use `with SessionLog(...) as log:` or call close()."""

    def __init__(self, root=None, name=None, tag="rec", meta=None):
        root = Path(config.POSITIONING_DIR if root is None else root)
        stem = time.strftime("%Y%m%d_%H%M%S") + f"_{tag}" + (f"_{name}" if name else "")
        for i in range(1, 100):                                   # two processes started in the same second
            self.path = root / (stem if i == 1 else f"{stem}_{i}")
            try:
                self.path.mkdir(parents=True, exist_ok=False); break
            except FileExistsError:
                continue
        else:
            raise RuntimeError(f"cannot create a session directory under {root}")
        self.n = 0
        self.t0_ms, self.wall0 = now_ms(), time.time()
        self.meta = {"schema": SCHEMA_VERSION, "tag": tag, "name": name, "started_wall": self.wall0,
                     "started_iso": time.strftime("%Y-%m-%dT%H:%M:%S%z"), "started_t_ms": self.t0_ms,
                     "argv": sys.argv, "cwd": os.getcwd(), "platform": platform.platform(),
                     "python": sys.version.split()[0], "git": _git_hash(),
                     "ports": {"cmd": CMD_PORT, "telem": TELEM_PORT, "state": STATE_PORT},
                     "config": _config_snapshot(), "meta": meta or {}}
        self._write_meta()
        self.f = open(self.path / LOG_NAME, "a", buffering=1, encoding="utf-8", newline="\n")   # line-buffered

    def _write_meta(self):
        (self.path / META_NAME).write_text(json.dumps(self.meta, indent=2, default=str) + "\n", encoding="utf-8")

    # --- one method per kind (schema.KINDS) ---
    def write(self, kind, data, t_ms=None, wall=None):
        if self.f is None:
            return
        self.f.write(json.dumps(row(kind, data, t_ms, wall), separators=(",", ":"), default=str) + "\n")
        self.n += 1

    def state(self, msg): self.write("state", msg)
    def telem(self, msg): self.write("telem", msg)
    def cmd(self, msg): self.write("cmd", msg)
    def truth(self, d): self.write("truth", d)

    def event(self, text, **kv):
        self.write("event", dict(text=text, **kv))

    def close(self):
        if self.f is not None:
            self.f.close(); self.f = None
            self.meta.update(ended_wall=time.time(), rows=self.n, duration_s=round(time.time() - self.wall0, 3))
            self._write_meta()

    def __enter__(self): return self
    def __exit__(self, *exc): self.close()
    def __bool__(self): return True
    def __repr__(self): return f"SessionLog({self.path}, rows={self.n})"


class NullLog:
    """Drop-in for SessionLog when --log was not given: every call is a no-op, `if log:` is False."""
    path, n = None, 0
    def write(self, kind, data, t_ms=None): pass
    def state(self, msg): pass
    def telem(self, msg): pass
    def cmd(self, msg): pass
    def truth(self, d): pass
    def event(self, text, **kv): pass
    def close(self): pass
    def __enter__(self): return self
    def __exit__(self, *exc): pass
    def __bool__(self): return False
    def __repr__(self): return "NullLog()"


def open_session(arg, tag, meta=None):
    """argparse helper: `--log` absent (None) -> NullLog; `--log` alone ("") -> unnamed session; `--log NAME` -> named."""
    if arg is None:
        return NullLog()
    log = SessionLog(name=arg or None, tag=tag, meta=meta)
    print(f"[session] recording to {log.path}")
    return log


# ------------------------------------------------------------------ reading
def iter_rows(session_dir):
    """Yield rows of log.jsonl in order. A truncated last line (killed mid-write) is skipped, not fatal."""
    path = Path(session_dir)
    path = path / LOG_NAME if path.is_dir() else path
    bad = 0
    with open(path, encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                yield json.loads(line)
            except ValueError:
                bad += 1
    if bad:
        print(f"[session] {path}: skipped {bad} unparsable line(s)")


def read_meta(session_dir):
    p = Path(session_dir) / META_NAME
    return json.loads(p.read_text(encoding="utf-8")) if p.exists() else {}


def _columns(rows):
    """List of row dicts -> {key: array}. Numeric scalars -> float (NaN for missing/None), uniform numeric lists
    -> (N, k) float with NaN rows, anything else -> python list. Generic so upstream fields can grow freely."""
    import numpy as np
    keys = []
    for r in rows:
        for k in r:
            if k not in keys: keys.append(k)
    out = {}
    for k in keys:
        vals = [r.get(k) for r in rows]
        present = [v for v in vals if v is not None]
        if all(isinstance(v, (int, float, bool)) for v in present):
            out[k] = np.array([np.nan if v is None else float(v) for v in vals], dtype=float)
        elif present and all(isinstance(v, (list, tuple)) and len(v) == len(present[0])
                             and all(isinstance(x, (int, float, bool)) for x in v) for v in present):
            w = len(present[0])
            out[k] = np.array([[np.nan] * w if v is None else [float(x) for x in v] for v in vals], dtype=float)
        else:
            out[k] = vals
    return out


def load(session_dir):
    """Whole session -> {"meta": dict, <kind>: {"t": recorder ms, "wall": s, "t_src": sender clock, <data fields>...}}.
    Every kind key is present (empty arrays when nothing was recorded). Needs numpy (imported here, lazily)."""
    import numpy as np
    by_kind = {k: [] for k in KINDS}
    for r in iter_rows(session_dir):
        by_kind.setdefault(r.get("kind"), []).append(r)
    out = {"meta": read_meta(session_dir)}
    for kind, rows in by_kind.items():
        d = _columns([r.get("data") or {} for r in rows]) if rows else {}
        if "t" in d:
            d["t_src"] = d.pop("t")
        d["t"] = np.array([r["t_ms"] for r in rows], dtype=float)
        d["wall"] = np.array([r["wall"] for r in rows], dtype=float)
        if kind == "event":
            d["rows"] = [r.get("data") or {} for r in rows]
        out[kind] = d
    return out


def summary(session_dir):
    d = load(session_dir)
    m = d["meta"]
    lines = [f"{session_dir}: tag={m.get('tag')} name={m.get('name')} started={m.get('started_iso')} git={m.get('git')} "
             f"venue={m.get('config', {}).get('VENUE')}"]
    for kind in KINDS:
        t = d[kind]["t"]
        if len(t) == 0:
            lines.append(f"  {kind:6s}     0 rows"); continue
        span = (t[-1] - t[0]) / 1000.0
        rate = f"{len(t) / span:5.1f} Hz" if span > 0 else "   -"
        extra = ""
        if kind == "state":
            b = d[kind].get("balloon"); p = d[kind].get("person")
            import numpy as np
            if isinstance(b, np.ndarray) and b.ndim == 2:
                extra += f"  balloon seen {int(np.isfinite(b[:, 0]).sum())}/{len(b)}"
            if isinstance(p, np.ndarray) and p.ndim == 2:
                extra += f"  person seen {int(np.isfinite(p[:, 0]).sum())}/{len(p)}"
        lines.append(f"  {kind:6s} {len(t):6d} rows  {span:7.1f} s  {rate}{extra}")
    return "\n".join(lines)


def main():
    if len(sys.argv) < 2:
        root = Path(config.POSITIONING_DIR)
        found = sorted(p for p in root.glob("*") if (p / LOG_NAME).exists()) if root.exists() else []
        print(__doc__); print(f"sessions under {root}:" if found else f"no sessions under {root}")
        for p in found: print(f"  {p}")
        return
    for s in sys.argv[1:]:
        print(summary(s))


if __name__ == "__main__":
    main()
