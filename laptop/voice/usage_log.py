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
