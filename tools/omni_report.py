"""Build the usage report the OMNI Live organisers want back (reply to the key e-mail by 2026-09-20 23:59 EDT).

  python tools/omni_report.py                       # ledger data/omni_usage.jsonl -> data/omni_report/
  python tools/omni_report.py --email you@x.com --extra-log other.jsonl

Runs the organisers' summarize_usage.py (tools/yibu/, unmodified) over the ledger, writes usage_summary.json and
usage_by_model_key_purpose.csv, and prints the e-mail text with team, project link, period, key suffixes and gaps.
Nothing here contains the key: the ledger and the summaries carry the last four characters only.
"""
import argparse, json, os, pathlib, subprocess, sys
ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT)); os.chdir(ROOT)
from laptop.voice.usage_log import DEFAULT_PATH

TEAM = "Raymond Zheng, Peter Rong, Clement Chung, David Wang"
PROJECT = "https://github.com/dnexdev/blimpy"

ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument("--log", default=DEFAULT_PATH); ap.add_argument("--out", default="data/omni_report")
ap.add_argument("--email", default="<application e-mail>")
ap.add_argument("--extra-log", action="append", default=[], help="other ledgers to merge (e.g. the organisers' examples' artifacts/yibu_api_calls.jsonl)")
ap.add_argument("--balance", action="store_true", help="ask the relay what the key has spent and when it expires (needs YIBU_API_KEY), then exit")
a = ap.parse_args()

if a.balance:
    import datetime, requests, zoneinfo
    key = os.environ.get("YIBU_API_KEY") or os.environ.get("OMNI_API_KEY") or sys.exit("set YIBU_API_KEY")
    H = {"Authorization": "Bearer " + key}
    sub = requests.get("https://yibuapi.com/v1/dashboard/billing/subscription", headers=H, timeout=20).json()
    today = datetime.date.today()
    use = requests.get(f"https://yibuapi.com/v1/dashboard/billing/usage?start_date={today - datetime.timedelta(days=7)}&end_date={today + datetime.timedelta(days=1)}",
                       headers=H, timeout=20).json()
    spent, limit = use.get("total_usage", 0) / 100.0, sub.get("hard_limit_usd")
    until = sub.get("access_until")
    et = zoneinfo.ZoneInfo("America/Toronto")
    print(f"key ...{key[-4:]}: spent {spent:.2f} of {limit} (relay units, {100 * spent / limit if limit else 0:.1f} %) in the last 7 days; "
          f"access until {datetime.datetime.fromtimestamp(until, et).strftime('%Y-%m-%d %H:%M %Z') if until else '?'}")
    sys.exit(0)

rows = []
for p in [a.log, *a.extra_log]:
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
print(f"""
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
record each. Purposes: pilot (the demo), omni_cli / live_test (development and protocol checks), focus_watch,
model_bench (choosing the focus-watch model). Gaps: a response cancelled by barge-in comes back from the server
without usage and is logged with usage null (counted under usage_missing_calls, not as zero); a session closed by a
network drop before response.done has no usage row for that response.
No prompts, audio, images or the key are in the ledger or the attachments.
""")
