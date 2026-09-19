"""Sponsored-API usage ledger (Huawei OMNI Live track). Every call Blimpy makes with the sponsored key (each realtime
response, each focus-watch HTTP call, each failed connect) is appended through the organisers' own
`yibu_audit.append_audit_record` (tools/yibu/, verbatim copy of their package) so `python tools/omni_report.py`
produces exactly the files they want back: usage_summary.json + usage_by_model_key_purpose.csv.

Ledger: data/omni_usage.jsonl (override with YIBU_AUDIT_LOG). Schema yibu_call_audit_v1: key suffix only ("...abcd"),
never prompts, audio or images. Missing usage stays null (unknown != zero).
"""
import json, os, pathlib, sys, threading

ROOT = pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools" / "yibu"))
import yibu_audit                                   # noqa: E402  (organisers' module, unmodified)

DEFAULT_PATH = os.environ.get("YIBU_AUDIT_LOG") or os.environ.get("OMNI_USAGE_LOG") or str(ROOT / "data" / "omni_usage.jsonl")
_lock = threading.Lock()


def key_suffix(key):
    return yibu_audit.key_suffix(key) if key else None


def record(model, key, purpose, endpoint, transport, ok, latency_ms=None, usage=None, error=None, path=None,
           call_id=None, status_code=None):
    """Append one call. `usage` is the provider's usage object (normalised by their code, kept verbatim as usage_raw).
    `endpoint` is the full URL. Returns the row written."""
    with _lock:
        return yibu_audit.append_audit_record(
            model=model, api_key=key or "", endpoint=endpoint, purpose=purpose, transport=transport, ok=bool(ok),
            latency_s=(latency_ms or 0.0) / 1000.0, response_json={"usage": usage} if usage else None,
            status_code=status_code, error=error, call_id=call_id, audit_log=path or DEFAULT_PATH)


RELAY_HTTP = os.environ.get("OMNI_HTTP", "https://yibuapi.com/v1").rsplit("/v1", 1)[0]


def relay_balance(key, timeout=15):
    """What the relay's dashboard says about this key: {"spent", "limit", "pct", "until" (epoch or None)}. Free (no
    model call). spent/limit are the dashboard's own units (the 200 limit): total_usage cents / 100."""
    import datetime, requests
    H = {"Authorization": "Bearer " + key}
    sub = requests.get(f"{RELAY_HTTP}/v1/dashboard/billing/subscription", headers=H, timeout=timeout).json()
    today = datetime.date.today()
    use = requests.get(f"{RELAY_HTTP}/v1/dashboard/billing/usage?start_date={today - datetime.timedelta(days=30)}"
                       f"&end_date={today + datetime.timedelta(days=1)}", headers=H, timeout=timeout).json()
    spent, limit = use.get("total_usage", 0) / 100.0, sub.get("hard_limit_usd") or 0
    return {"spent": spent, "limit": limit, "pct": 100.0 * spent / limit if limit else 0.0, "until": sub.get("access_until")}


def relay_sessions(key, timeout=15):
    """The relay's own per-call log for this key (one row per realtime SESSION, one per HTTP call), oldest first:
    [{"t": epoch, "model", "quota", "units", "audio_in_s", "audio_out", "text_in", "text_out", "prompt", "completion", "use_s"}].
    units = quota / 250 000 (the dashboard's unit; ~0.5 % off the dashboard total, good enough to see what a session cost)."""
    import requests
    r = requests.get(f"{RELAY_HTTP}/api/log/token?key={key}&page_size=500", headers={"Authorization": "Bearer " + key}, timeout=timeout)
    d = r.json().get("data") or []
    rows = d if isinstance(d, list) else (d.get("items") or d.get("data") or [])
    out = []
    for x in sorted(rows, key=lambda x: x.get("created_at", 0)):
        try: o = json.loads(x.get("other") or "{}")
        except ValueError: o = {}
        out.append({"t": x.get("created_at"), "model": x.get("model_name"), "quota": x.get("quota", 0), "units": x.get("quota", 0) / 250_000.0,
                    "audio_in_s": o.get("audio_input", 0) / 100.0, "audio_out": o.get("audio_output", 0), "text_in": o.get("text_input", 0),
                    "text_out": o.get("text_output", 0), "prompt": x.get("prompt_tokens", 0), "completion": x.get("completion_tokens", 0),
                    "use_s": x.get("use_time", 0)})
    return out


def reconcile(ledger_rows, relay_rows, slack_s=120):
    """Ledger against the relay's own bill (relay_sessions), per model. The relay has one row per realtime SESSION, the
    ledger one per response, so rows are matched by time, not counted: a relay row is `unlogged` when no ledger row of
    its model falls inside its span (created_at - use_s .. created_at, +- slack_s). Those are calls made where this
    ledger was not (another machine, the organisers' examples run elsewhere, a session that died before response.done).
    Returns {model: {"relay_rows", "relay_prompt", "relay_completion", "relay_units", "ledger_calls", "ledger_in",
    "ledger_out", "ledger_missing", "unlogged": [relay rows]}}."""
    import datetime
    out, stamps = {}, {}
    def slot(m):
        return out.setdefault(m, {"relay_rows": 0, "relay_prompt": 0, "relay_completion": 0, "relay_units": 0.0, "ledger_calls": 0,
                                  "ledger_in": 0, "ledger_out": 0, "ledger_missing": 0, "unlogged": []})
    for r in ledger_rows:
        s = slot(r.get("model")); s["ledger_calls"] += 1
        s["ledger_in"] += r.get("input_tokens") or 0; s["ledger_out"] += r.get("output_tokens") or 0
        if not r.get("usage_reported"): s["ledger_missing"] += 1
        if r.get("timestamp_utc"):
            stamps.setdefault(r.get("model"), []).append(datetime.datetime.fromisoformat(r["timestamp_utc"]).timestamp())
    for x in relay_rows:
        s = slot(x.get("model")); s["relay_rows"] += 1; s["relay_units"] += x.get("units") or 0.0
        s["relay_prompt"] += x.get("prompt") or 0; s["relay_completion"] += x.get("completion") or 0
        t = x.get("t") or 0; lo, hi = t - (x.get("use_s") or 0) - slack_s, t + slack_s
        if not any(lo <= ts <= hi for ts in stamps.get(x.get("model"), [])): s["unlogged"].append(x)
    return out


def summary(path=None):
    """Totals per (model, purpose) for a quick look:  python -m laptop.voice.usage_log"""
    p = path or DEFAULT_PATH
    tot = {}
    if not os.path.exists(p): return tot
    for line in open(p, encoding="utf-8"):
        if not line.strip(): continue
        r = json.loads(line); k = (r.get("model"), r.get("purpose"))
        t = tot.setdefault(k, {"calls": 0, "fail": 0, "in": 0, "out": 0, "missing": 0})
        t["calls"] += 1; t["fail"] += 0 if r.get("ok") else 1
        if not r.get("usage_reported"): t["missing"] += 1
        t["in"] += r.get("input_tokens") or 0; t["out"] += r.get("output_tokens") or 0
    return tot


if __name__ == "__main__":
    print(f"ledger: {DEFAULT_PATH}")
    for (m, pur), t in summary().items():
        print(f"{m:34s} {pur:20s} calls {t['calls']:4d} (fail {t['fail']})  in {t['in']:8d}  out {t['out']:8d}  usage-missing {t['missing']}")
