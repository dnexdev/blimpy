"""Replay recorded voice sessions through the CURRENT "who is that for" code. No key, no credits.

  python tools/addressee_backtest.py                          # every session under data/voice_sessions + tools/fixtures/addressee_cases.jsonl
  python tools/addressee_backtest.py --label <session dir>    # say who each turn was for (y / n / s skip / q quit; hears it with --play)
  python tools/addressee_backtest.py --set accept=0.7,0.8 --set weights.engaged=0.6      # try parameters before touching config
  python tools/addressee_backtest.py --judge off              # what happens with no judge at all (key expired, relay down)

Each recorded turn carries the context the addressee module saw (addressee.Ctx), so the cues and the thresholds are
recomputed exactly. The judge: `recorded` (default) reuses the verdict the judge gave on the day, as long as the question
put to it is still the same (hash of the prompt); a turn that NOW needs the judge but has no reusable verdict is
reported as "needs judge" and settled by the midpoint. `relay` / `ollama` ask for real (relay costs a few hundred text
tokens per turn; only the turns without a reusable verdict are asked, and the answers are saved into the session, so the
next replay is free again: run it once after changing the judge's prompt). Truth = the human label when there is one, else the verdict of the day
(then the report shows what CHANGED, not what is right: label your sessions).
Exit code 1 when a labelled turn is judged wrong."""
import argparse, glob, json, pathlib, sys
ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from laptop.voice import addressee as ad, omni                      # noqa: E402
from laptop.voice.session_rec import DEFAULT_DIR                     # noqa: E402

FIXTURES = ROOT / "tools" / "fixtures" / "addressee_cases.jsonl"


def load(path):
    p = pathlib.Path(path); f = p / "turns.jsonl" if p.is_dir() else p
    rows = [json.loads(l) for l in f.read_text(encoding="utf-8").splitlines() if l.strip()] if f.exists() else []
    return f, rows


def ctx_of(row):
    c = dict(row["ctx"]); c["history"] = tuple(tuple(h) for h in c.get("history") or ())
    return ad.Ctx(**{k: v for k, v in c.items() if k in ad.Ctx.__dataclass_fields__})


def replay(row, A, judge, base=None):
    """-> (ok, why, note). note: '' | 'judge reused' | 'needs judge' | 'asked'."""
    import base64
    ctx = ctx_of(row)
    pics = [base / f for f in row.get("frames") or [] if base is not None and (base / f).exists()][:int(A.P["judge_frames"])]
    if row.get("why", "").startswith("press-to-talk"): return True, "press-to-talk", ""
    hard = omni._hard(A, ctx, False, row.get("policy") or "smart")
    if hard: return hard[0], hard[1], ""
    dec = A.decide(ctx)
    if dec.ok is not None: return dec.ok, dec.why, ""
    p = why = None; note = "needs judge"
    j = row.get("judge") or {}
    if judge != "off" and j.get("p") is not None and j.get("hash") == omni.session_prompt_hash(ctx, A.P["names"], j.get("pictures", 0)):
        p, why, note = j["p"], j.get("why", ""), "judge reused"
    elif judge not in ("off", "recorded"):
        J = ad.make_judge(judge, names=A.P["names"], timeout=15); fr = [base64.b64encode(f.read_bytes()).decode() for f in pics] if getattr(J, "wants_frames", False) else []
        p, why = J.ask(ctx, fr) if fr else J.ask(ctx); note = "asked"
        if p is not None: row["judge"] = {"p": p, "why": why, "hash": omni.session_prompt_hash(ctx, A.P["names"], len(fr)), "pictures": len(fr)}; row["_fresh"] = True   # kept: the next replay is free
    elif judge == "off": note = ""
    dec = A.resolve(ctx, dec, p, why or "")
    return dec.ok, dec.why, note


def label(path, play):
    f, rows = load(path); wav = f.parent / "mic.wav"
    for r in rows:
        if r.get("who") != "user" or r.get("label") is not None or not r.get("ctx", {}).get("text"): continue
        print(f"\n  [{r.get('t0')} s] \"{r['ctx']['text']}\"\n  on the day: {'FOR BLIMPY' if r['verdict'] else 'ignored'} ({r.get('why')})")
        if play and wav.exists() and r.get("t0") is not None: play_clip(wav, max(0.0, r["t0"] - 2.0), (r.get("t1") or r["t0"] + 4.0))
        a = input("  was it said TO Blimpy? [y/n/s/q] ").strip().lower()
        if a == "q": break
        if a in ("y", "n"): r["label"] = a == "y"
    f.write_text("".join(json.dumps(r, ensure_ascii=False) + "\n" for r in rows), encoding="utf-8")
    print(f"[backtest] labels saved in {f}")


def play_clip(wav, t0, t1):
    import wave, numpy as np, sounddevice as sd
    with wave.open(str(wav)) as w:
        w.setpos(min(w.getnframes(), int(t0 * w.getframerate()))); pcm = w.readframes(int((t1 - t0) * w.getframerate()))
        sd.play(np.frombuffer(pcm, np.int16), w.getframerate()); sd.wait()


def set_param(P, spec):
    k, v = spec.split("=", 1); val = tuple(float(x) for x in v.split(",")); val = val if len(val) > 1 else val[0]
    if k.startswith("weights."): P.setdefault("weights", {})[k.split(".", 1)[1]] = val
    else: P[k] = val


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("paths", nargs="*", help="session directories or .jsonl files (default: all sessions + the fixtures)")
    ap.add_argument("--label", metavar="SESSION"); ap.add_argument("--play", action="store_true", help="with --label: play each turn from mic.wav")
    ap.add_argument("--judge", default="recorded", choices=("recorded", "off", "relay", "ollama"))
    ap.add_argument("--set", action="append", default=[], metavar="KEY=VALUE", help="addressee.DEFAULTS override, e.g. accept=0.7,0.8 or weights.form=0.5")
    ap.add_argument("-v", "--verbose", action="store_true", help="print every turn, not only the changed / wrong ones")
    a = ap.parse_args()
    if a.label: label(a.label, a.play); sys.exit(0)
    P = {}
    for s in a.set: set_param(P, s)
    paths = a.paths or sorted(glob.glob(str(DEFAULT_DIR / "*"))) + [str(FIXTURES)]
    n = right = wrong = changed = unlabelled = need = 0
    for path in paths:
        f, every = load(path)
        rows = [r for r in every if r.get("who") == "user" and r.get("ctx", {}).get("text")]
        if not rows: continue
        names = None
        meta = f.parent / "meta.json"
        if meta.exists(): names = (json.loads(meta.read_text()).get("params") or {}).get("names")
        A = ad.Addressee(dict(P, names=tuple(names)) if names else dict(P, names=omni.NAME_GATE["words"]))
        print(f"\n{f.parent.name if f.name == 'turns.jsonl' else f.name}  ({len(rows)} turns)")
        for r in rows:
            ok, why, note = replay(r, A, a.judge, f.parent); n += 1; need += note == "needs judge"
            truth = r.get("label"); day = r.get("verdict")
            if truth is None:
                unlabelled += 1; tag = "changed" if ok != day else "same"; changed += ok != day
            else:
                tag = "ok" if ok == truth else "WRONG"; right += ok == truth; wrong += ok != truth
            if a.verbose or tag in ("changed", "WRONG"):
                print(f"  {tag:7s} {'YES' if ok else 'no ':3s} \"{r['ctx']['text'][:70]}\"  [{why}]{'  (' + note + ')' if note else ''}"
                      + (f"   truth: {'YES' if truth else 'no'}" if truth is not None else f"   on the day: {'YES' if day else 'no'}"))
        if any(r.pop("_fresh", False) for r in rows): f.write_text("".join(json.dumps(r, ensure_ascii=False) + "\n" for r in every), encoding="utf-8")
    print(f"\n[backtest] {n} turns: labelled {right + wrong} ({right} right, {wrong} WRONG), unlabelled {unlabelled} ({changed} differ from the day)"
          + (f", {need} would need a fresh judge verdict (settled by the midpoint here)" if need else ""))
    sys.exit(1 if wrong else 0)
