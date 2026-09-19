"""Benchmark the local intent parser (laptop/voice/intent.py) across Ollama models: accuracy + latency.

  python tools/intent_test.py                          # the model pinned in intent.py
  python tools/intent_test.py --models qwen2.5:1.5b qwen2.5:3b qwen2.5:7b
  python tools/intent_test.py --no-fast                # force everything through the LLM (skip the regex shortcuts)

A case passes when the intent matches and every expected field matches (numbers within 10 %).
Phrases are deliberately conversational / mis-heard the way whisper returns them (no punctuation, filler words).
"""
import argparse, os, sys, time
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from laptop.voice import intent

# (spoken text, expected intent, expected fields)
CASES = [
    ("hey blimpy follow me", "follow_me", {}),
    ("come with me", "follow_me", {}),
    ("okay blimpy you can follow me now", "follow_me", {}),
    ("stop", "hover", {}),
    ("stay right there", "hover", {}),
    ("hold on a second", "hover", {}),
    ("come here", "go_to", {"target": "me"}),
    ("can you come over to me please", "go_to", {"target": "me"}),
    ("go home", "go_to", {"target": "home"}),
    ("go back to where you started", "go_to", {"target": "home"}),
    ("go show the judges what you can do", "go_to", {"target": "judges"}),
    ("float over to the judges table", "go_to", {"target": "judges"}),
    ("wander around for a bit", "wander", {}),
    ("go explore the room", "wander", {}),
    ("go up", "altitude", {"direction": "up"}),
    ("a little higher please", "altitude", {"direction": "up"}),
    ("come down a bit", "altitude", {"direction": "down"}),
    ("you're too high", "altitude", {"direction": "down"}),
    ("turn around", "rotate", {"degrees": 180}),
    ("spin", "rotate", {"degrees": 360}),
    ("turn left", "rotate", {"degrees": 90}),
    ("turn right ninety degrees", "rotate", {"degrees": -90}),
    ("could you turn to your right about forty five degrees", "rotate", {"degrees": -45}),
    ("rotate thirty degrees to the left", "rotate", {"degrees": 30}),
    ("set a timer for twenty minutes", "timer", {"minutes": 20}),
    ("remind me in ninety seconds", "timer", {"minutes": 1.5}),
    ("timer for five minutes please", "timer", {"minutes": 5}),
    ("start a pomodoro", "pomodoro", {}),
    ("let's do a pomodoro with fifty minutes of work and ten of break", "pomodoro", {"work_min": 50, "break_min": 10}),
    ("make sure I keep working", "focus_guard", {"enabled": True}),
    ("stop watching me work", "focus_guard", {"enabled": False}),
    ("do a little dance", "mood", {"mood": "dance"}),
    ("what's your name", "say", {}),
    ("how are you doing today", "say", {}),
    ("um so anyway I told him that the deadline was friday", "none", {}),
]


def field_ok(got, want):
    if isinstance(want, bool): return got is want or got == want
    if isinstance(want, (int, float)):
        try: return abs(float(got) - want) <= 0.1 * abs(want) + 1e-6
        except (TypeError, ValueError): return False
    return str(got).lower() == str(want).lower()


def run(model, use_fast):
    ok, lat, fails = 0, [], []
    for text, want_intent, want_fields in CASES:
        t = time.time()
        it = (intent.fast_intent(text) if use_fast else None) or intent.parse_intent(text, model=model)
        dt = time.time() - t; lat.append(dt)
        good = it.get("intent") == want_intent and all(field_ok(it.get(k), v) for k, v in want_fields.items())
        ok += good
        if not good: fails.append((text, want_intent, want_fields, {k: it.get(k) for k in ("intent", *want_fields)}))
    return ok, lat, fails


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--models", nargs="*", default=[intent.MODEL])
    ap.add_argument("--no-fast", action="store_true")
    ap.add_argument("-v", action="store_true", help="show failures")
    args = ap.parse_args()
    for model in args.models:
        intent.warmup(model)
        ok, lat, fails = run(model, not args.no_fast)
        llm_lat = sorted(l for l in lat if l > 0.05)          # regex hits are ~0 s; report the model's own latency
        print(f"{model:14s} {ok}/{len(CASES)} correct   LLM latency median {llm_lat[len(llm_lat)//2]:.2f}s  "
              f"max {max(llm_lat):.2f}s  ({len(llm_lat)} LLM calls, {len(lat) - len(llm_lat)} regex)")
        if args.v:
            for text, wi, wf, got in fails:
                print(f"   FAIL {text!r}: want {wi} {wf} got {got}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
