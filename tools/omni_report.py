"""Build the usage report the OMNI Live organisers want back (reply to the key e-mail by 2026-09-20 23:59 EDT).

  python tools/omni_report.py                       # ledger data/omni_usage.jsonl -> data/omni_report/
  python tools/omni_report.py --email you@x.com --extra-log other.jsonl
  python tools/omni_report.py --balance             # what the key has spent of its limit, and when the relay cuts it off
  python tools/omni_report.py --sessions            # the relay's own bill per session: seconds of audio sent -> units
  python tools/omni_report.py --reconcile           # also check the ledger against the relay's bill: what was never logged

Runs the organisers' summarize_usage.py (tools/yibu/, unmodified) over the ledger, writes usage_summary.json and
usage_by_model_key_purpose.csv, and prints the e-mail text (also saved as email_draft.txt) with team, project link,
period, key suffixes and gaps. Calls made on another machine are in that machine's data/omni_usage.jsonl: copy it over
and pass it with --extra-log. The organisers' examples' own ledger (tools/yibu/artifacts/) is merged when it exists.
Nothing here contains the key: the ledger and the summaries carry the last four characters only.
"""
import argparse, json, os, pathlib, subprocess, sys
ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT)); os.chdir(ROOT)
from laptop.voice.usage_log import DEFAULT_PATH, reconcile, relay_balance, relay_sessions

TEAM = "Raymond Zheng, Peter Rong, Clement Chung, David Wang"
PROJECT = "https://github.com/dnexdev/blimpy"
EXAMPLES_LOG = ROOT / "tools" / "yibu" / "artifacts" / "yibu_api_calls.jsonl"     # where the organisers' examples log by default

ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument("--log", default=DEFAULT_PATH); ap.add_argument("--out", default="data/omni_report")
ap.add_argument("--email", default="<application e-mail>")
ap.add_argument("--extra-log", action="append", default=[], help="other ledgers to merge (e.g. the organisers' examples' artifacts/yibu_api_calls.jsonl)")
ap.add_argument("--balance", action="store_true", help="ask the relay what the key has spent and when it expires (needs YIBU_API_KEY), then exit")
ap.add_argument("--sessions", action="store_true", help="the relay's per-session bill (audio seconds, units), then exit")
ap.add_argument("--reconcile", action="store_true", help="compare the ledger with the relay's own bill (needs YIBU_API_KEY); relay calls with no ledger row go in the e-mail's gaps")
a = ap.parse_args()

if a.balance or a.sessions:
    import datetime, zoneinfo
    key = os.environ.get("YIBU_API_KEY") or os.environ.get("OMNI_API_KEY") or sys.exit("set YIBU_API_KEY")
    et = zoneinfo.ZoneInfo("America/Toronto")
    if a.sessions:
        rows = relay_sessions(key)
        print(f"{'when (Toronto)':16} {'model':28} {'units':>6} {'audio in':>8} {'audio out':>9} {'text in':>7} {'text out':>8} {'session':>7}")
        for r in rows:
            print(f"{datetime.datetime.fromtimestamp(r['t'], et).strftime('%m-%d %H:%M:%S'):16} {r['model']:28} {r['units']:6.3f} {r['audio_in_s']:7.1f}s "
                  f"{r['audio_out']:8d}t {r['text_in']:6d}t {r['text_out']:7d}t {r['use_s']:6d}s")
        print(f"{len(rows)} rows, {sum(r['units'] for r in rows):.2f} units; the realtime model bills the audio you SEND (0.77 units per minute), "
              f"not the prompt")
    b = relay_balance(key)
    print(f"key ...{key[-4:]}: spent {b['spent']:.2f} of {b['limit']} ({b['pct']:.1f} %); "
          f"access until {datetime.datetime.fromtimestamp(b['until'], et).strftime('%Y-%m-%d %H:%M %Z') if b['until'] else '?'}")
    sys.exit(0)

rows = []
logs = [a.log, *a.extra_log]
if EXAMPLES_LOG.exists() and not any(os.path.exists(p) and os.path.samefile(p, EXAMPLES_LOG) for p in logs): logs.append(str(EXAMPLES_LOG))
for p in logs:
    if os.path.exists(p): rows += [l for l in open(p, encoding="utf-8") if l.strip()]
    else: print(f"[report] no ledger at {p}")
if not rows: sys.exit("[report] nothing to report: the ledger is empty")
os.makedirs(a.out, exist_ok=True)
merged = os.path.join(a.out, "omni_usage_merged.jsonl")
open(merged, "w", encoding="utf-8").write("".join(rows))
subprocess.run([sys.executable, str(ROOT / "tools" / "yibu" / "summarize_usage.py"), "--log", merged, "--out-dir", a.out], check=True)

recs = [json.loads(l) for l in rows]
ts = sorted(r["timestamp_utc"] for r in recs if r.get("timestamp_utc"))
suffixes = sorted({r.get("key_suffix") for r in recs if r.get("key_suffix")})
s = json.load(open(os.path.join(a.out, "usage_summary.json"), encoding="utf-8"))
t = s["totals"]
purposes = ", ".join(sorted({r.get("purpose") or "[missing]" for r in recs}))

gap = ""
if a.reconcile:
    import datetime, zoneinfo
    key = os.environ.get("YIBU_API_KEY") or os.environ.get("OMNI_API_KEY") or sys.exit("--reconcile needs YIBU_API_KEY")
    et = zoneinfo.ZoneInfo("America/Toronto")
    rec = reconcile(recs, relay_sessions(key))
    print(f"\n{'model':34} {'relay rows':>10} {'unlogged':>8} {'relay in':>9} {'relay out':>9} {'ledger calls':>12} {'ledger in':>9} {'ledger out':>10} {'no usage':>8}")
    for m, x in sorted(rec.items(), key=lambda kv: str(kv[0])):
        print(f"{str(m):34} {x['relay_rows']:10d} {len(x['unlogged']):8d} {x['relay_prompt']:9d} {x['relay_completion']:9d} "
              f"{x['ledger_calls']:12d} {x['ledger_in']:9d} {x['ledger_out']:10d} {x['ledger_missing']:8d}")
    un = sorted((u for x in rec.values() for u in x["unlogged"]), key=lambda u: u["t"] or 0)
    for u in un:
        print(f"  unlogged: {datetime.datetime.fromtimestamp(u['t'] or 0, et).strftime('%m-%d %H:%M:%S')} {u['model']} "
              f"{u['units']:.3f} units, {u['prompt']} + {u['completion']} tokens")
    n_relay = sum(x["relay_rows"] for x in rec.values())
    gap = (f"Checked against the relay's own log for this key ({n_relay} rows): "
           + (f"{len(un)} relay rows ({sum(u['units'] for u in un):.2f} units, {sum(u['prompt'] for u in un)} prompt + "
              f"{sum(u['completion'] for u in un)} completion tokens by the relay's count) have no record in our ledger "
              f"(calls made outside the logged application)." if un else "every relay row has a ledger record in its time span.") + "\n")

draft = f"""
================ e-mail draft (reply to the key e-mail; attach {a.out}/usage_summary.json and usage_by_model_key_purpose.csv)

Subject: OMNI Live usage report - Blimpy

Team: {TEAM}
Project: {PROJECT}
Application e-mail: {a.email}
Reporting period (UTC): {ts[0] if ts else '?'} to {ts[-1] if ts else '?'}
Key suffix(es): {', '.join(suffixes)}

Calls: {t['calls']} ({t['ok']} ok, {t['failed']} failed); tokens in {t['input_tokens']}, out {t['output_tokens']}, total {t['total_tokens']};
calls without usage from upstream: {t['usage_missing_calls']}.

How the calls were logged: every call from our application goes through your yibu_audit.append_audit_record
(unmodified copy in our repo). Realtime sessions are logged one record per response (each response.done with its
usage), plus one failed record per connection attempt that did not reach a session. Focus-watch HTTP calls are one
record each. Purposes in the ledger: {purposes}. Gaps: a response cancelled by barge-in comes back from the server
without usage and is logged with usage null (counted under usage_missing_calls, not as zero); a session closed by a
network drop before response.done has no usage row for that response.
{gap}No prompts, audio, images or the key are in the ledger or the attachments.
"""
print(draft)
open(os.path.join(a.out, "email_draft.txt"), "w", encoding="utf-8").write(draft.lstrip())
