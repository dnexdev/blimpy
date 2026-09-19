"""Loads <repo>/.env into os.environ on first import (every entry point imports `laptop` before it reads a key).
KEY=value lines, # comments, optional quotes, optional `export `. The shell wins: a variable already set is never
overwritten. OMNI_BASE_URL (https://host/v1) fills in OMNI_HTTP and OMNI_URL (wss://host/v1/realtime) when those are unset."""
import os, pathlib


def load_env(path=None):
    p = pathlib.Path(path) if path else pathlib.Path(__file__).resolve().parents[1] / ".env"
    if p.is_file():
        for line in p.read_text(encoding="utf-8").splitlines():
            line = line.strip()
            if not line or line.startswith("#") or "=" not in line: continue
            k, v = line.removeprefix("export ").split("=", 1)
            v = v.strip()
            if len(v) >= 2 and v[0] == v[-1] and v[0] in "\"'": v = v[1:-1]
            elif " #" in v: v = v.split(" #", 1)[0].rstrip()
            if v: os.environ.setdefault(k.strip(), v)
    base = os.environ.get("OMNI_BASE_URL", "").rstrip("/")
    if base:
        os.environ.setdefault("OMNI_HTTP", base)
        os.environ.setdefault("OMNI_URL", base.replace("https://", "wss://", 1).replace("http://", "ws://", 1) + "/realtime")


load_env()
