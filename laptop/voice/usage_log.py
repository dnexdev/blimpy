"""Sponsored-API usage ledger (Huawei OMNI Live track): one JSON line per call, same fields as the organisers'
`yibu_audit.append_audit_record` so their `summarize_usage.py --log data/omni_usage.jsonl` produces the report they
want back (usage_summary.json + usage_by_model_key_purpose.csv, due at the end of the event day).

Never logs the key (only "...last4"), prompts, audio or images. Missing usage stays null (unknown != zero).
"""
import json, os, threading, time, uuid

DEFAULT_PATH = os.environ.get("YIBU_AUDIT_LOG") or os.environ.get("OMNI_USAGE_LOG") or "data/omni_usage.jsonl"
_lock = threading.Lock()


def key_suffix(key):
    return "..." + key[-4:] if key else None


def record(model, key, purpose, endpoint, transport, ok, latency_ms=None, usage=None, error=None, path=None,
           call_id=None):
    """Append one call. `usage` is the provider's usage object (kept verbatim as usage_raw and normalised)."""
    u = usage or {}
    inp = u.get("input_tokens", u.get("prompt_tokens"))
    out = u.get("output_tokens", u.get("completion_tokens"))
    tot = u.get("total_tokens")
    derived = False
    if tot is None and inp is not None and out is not None:
        tot, derived = inp + out, True
    row = {
        "call_id": call_id or uuid.uuid4().hex,
        "ts": time.strftime("%Y-%m-%dT%H:%M:%S", time.gmtime()) + "Z",
        "model": model, "key_suffix": key_suffix(key), "purpose": purpose,
        "endpoint": endpoint, "transport": transport, "success": bool(ok),
        "latency_ms": None if latency_ms is None else round(float(latency_ms), 1),
        "input_tokens": inp, "output_tokens": out, "total_tokens": tot, "total_tokens_derived": derived,
        "usage_raw": usage if usage else None, "error": (str(error)[:300] if error else None),
    }
    p = path or DEFAULT_PATH
    with _lock:
        os.makedirs(os.path.dirname(p) or ".", exist_ok=True)
        with open(p, "a", encoding="utf-8") as f:
            f.write(json.dumps(row) + "\n")
    return row


def summary(path=None):
    """Totals per (model, purpose) for a quick look:  python -m laptop.voice.usage_log"""
    p = path or DEFAULT_PATH
    tot = {}
    if not os.path.exists(p): return tot
    for line in open(p, encoding="utf-8"):
        if not line.strip(): continue
        r = json.loads(line); k = (r.get("model"), r.get("purpose"))
        t = tot.setdefault(k, {"calls": 0, "fail": 0, "in": 0, "out": 0, "missing": 0})
        t["calls"] += 1; t["fail"] += 0 if r.get("success") else 1
        if r.get("input_tokens") is None and r.get("output_tokens") is None: t["missing"] += 1
        t["in"] += r.get("input_tokens") or 0; t["out"] += r.get("output_tokens") or 0
    return tot


if __name__ == "__main__":
    for (m, pur), t in summary().items():
        print(f"{m:34s} {pur:20s} calls {t['calls']:4d} (fail {t['fail']})  in {t['in']:8d}  out {t['out']:8d}  usage-missing {t['missing']}")
