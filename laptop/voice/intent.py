"""Sentence -> intent JSON, using a local LLM through Ollama (structured output, always valid JSON).

  python -m laptop.voice.intent "follow me around"          # one-off test
  python -m laptop.voice.intent                             # interactive

Intents (the behavior layer in laptop/control/behaviors.py consumes exactly these):
  follow_me                      follow the speaker at ~1.5 m
  hover                          stop and hold position
  wander                         drift around slowly
  go_to        {target}          "judges" | "home" | "me"
  rotate       {degrees}         +CCW / -CW, e.g. 90, -180, 360
  altitude     {direction}       "up" | "down"
  timer        {minutes}         set a timer
  pomodoro     {work_min, break_min}
  focus_guard  {enabled}         nag the user if they leave / look away
  mood         {mood}            "happy" | "dance" | "sleepy"
  say          {text}            just say something back (small talk, questions)
  none                           not a command
"""
import json, os, re, sys
import ollama

# tools/intent_test.py, 35 phrases, RTX 5080 (Ollama on the GPU!): 7b 35/35 at 0.8 s median (1.5 s max),
# 3b 31/35 at 0.5 s, 1.5b 28/35. On CPU the 7b takes ~5 s: set BLIMPY_LLM=qwen2.5:3b then.
MODEL = os.environ.get("BLIMPY_LLM", "qwen2.5:7b")

# Every field is present in every answer (null when unused). With optional fields the smaller models simply skip
# them ("intent":"timer" with no minutes); forcing the keys makes them read the number out of the sentence.
def _opt(t): return {"anyOf": [t, {"type": "null"}]}
SCHEMA = {
    "type": "object",
    "properties": {
        "intent": {"type": "string", "enum": ["follow_me", "hover", "wander", "go_to", "rotate", "altitude", "timer",
                                              "pomodoro", "focus_guard", "mood", "say", "none"]},
        "target": _opt({"type": "string", "enum": ["judges", "home", "me"]}),
        "degrees": _opt({"type": "number"}),
        "direction": _opt({"type": "string", "enum": ["up", "down"]}),
        "minutes": _opt({"type": "number"}),
        "work_min": _opt({"type": "number"}),
        "break_min": _opt({"type": "number"}),
        "enabled": _opt({"type": "boolean"}),
        "mood": _opt({"type": "string", "enum": ["happy", "dance", "sleepy"]}),
        "reply": {"type": "string"},
    },
    "required": ["intent", "target", "degrees", "direction", "minutes", "work_min", "break_min", "enabled", "mood", "reply"],
}

SYSTEM = """You are Blimpy, a small friendly helium-balloon robot that floats around a room and follows people.
Convert the user's spoken sentence into ONE intent JSON object. Rules:
- Pick the single best intent. Fill only the fields that intent needs.
- rotate: degrees positive = counter-clockwise / LEFT, negative = clockwise / RIGHT ("thirty degrees to the left" = 30,
  "to your right about forty five" = -45). "turn around" = 180. "spin" = 360.
- timer: minutes as a number (e.g. "ninety seconds" -> 1.5). pomodoro defaults: work_min 25, break_min 5.
- go_to: "judges" (the judges' table), "home" (the starting spot: "go back to where you started"), or "me"
  ("come here", "come over to me"). Always fill target.
- altitude: "too high" / "lower" -> down; "too low" / "higher" -> up. Always fill direction.
- focus_guard: "keep me working" / "watch me" -> enabled true; "stop watching" / "leave me alone" -> enabled false.
- Fields the intent does not use are null.
- If the user is just chatting or asking a question, use intent "say" and answer briefly in "reply".
- If it is clearly not addressed to you or is noise, use intent "none".
- "reply" is ALWAYS a short spoken confirmation (max 12 words), in character, no emojis."""

_NULLS = {"target": None, "degrees": None, "direction": None, "minutes": None, "work_min": None, "break_min": None, "enabled": None, "mood": None}
FEW_SHOT = [
    ("hey blimpy follow me", {"intent": "follow_me", "reply": "On it, right behind you."}),
    ("stop", {"intent": "hover", "reply": "Holding here."}),
    ("set a timer for twenty minutes", {"intent": "timer", "minutes": 20, "reply": "Timer set, twenty minutes."}),
    ("start a pomodoro", {"intent": "pomodoro", "work_min": 25, "break_min": 5, "reply": "Pomodoro started, focus time."}),
    ("turn right ninety degrees", {"intent": "rotate", "degrees": -90, "reply": "Turning right."}),
    ("rotate twenty degrees to the left", {"intent": "rotate", "degrees": 20, "reply": "Turning left a little."}),
    ("go back to where you started", {"intent": "go_to", "target": "home", "reply": "Heading home."}),
    ("you're too high", {"intent": "altitude", "direction": "down", "reply": "Coming down."}),
    ("go show the judges", {"intent": "go_to", "target": "judges", "reply": "Heading to the judges."}),
    ("make sure I keep working", {"intent": "focus_guard", "enabled": True, "reply": "Focus guard on. No slacking."}),
    ("what's your name", {"intent": "say", "reply": "I'm Blimpy, your floating study buddy."}),
    ("go higher", {"intent": "altitude", "direction": "up", "reply": "Going up."}),
    ("come down a bit", {"intent": "altitude", "direction": "down", "reply": "Coming down."}),
    ("do a little dance", {"intent": "mood", "mood": "dance", "reply": "Dance mode, watch this."}),
    ("drift around for a while", {"intent": "wander", "reply": "Wandering. I'll be around."}),
]


# Instant path for the phrases people actually say; anything else goes to the model.
FAST = [
    # focus guard first: "stop watching me" must not fall into the "stop" -> hover rule below
    (r"\b(stop|quit|no more) (watching|nagging|bugging|checking on)\b|\bleave me alone\b",
     {"intent": "focus_guard", "enabled": False, "reply": "Okay, I'll leave you be."}),
    (r"\b(keep me|make sure i keep|watch me|make sure i'?m) (working|focused|on task)\b|\bfocus (guard|mode)\b",
     {"intent": "focus_guard", "enabled": True, "reply": "Focus guard on. No slacking."}),
    (r"\b(follow|come with) me\b|\bfollow\b", {"intent": "follow_me", "reply": "On it, right behind you."}),
    (r"\b(stop|hold( still| on| position)?|stay( there| here)?|freeze|hover|wait)\b", {"intent": "hover", "reply": "Holding here."}),
    (r"\bcome (here|to me|over)\b", {"intent": "go_to", "target": "me", "reply": "Coming over."}),
    (r"\b(go )?home\b", {"intent": "go_to", "target": "home", "reply": "Heading home."}),
    (r"\bjudges?\b", {"intent": "go_to", "target": "judges", "reply": "Heading to the judges."}),
    (r"\b(wander|roam|explore|drift around)\b", {"intent": "wander", "reply": "Wandering. I'll be around."}),
    (r"\b(go |float |fly )?(up|higher)\b", {"intent": "altitude", "direction": "up", "reply": "Going up."}),
    (r"\b(go |come |float |fly )?(down|lower)\b", {"intent": "altitude", "direction": "down", "reply": "Coming down."}),
    (r"\bdance\b", {"intent": "mood", "mood": "dance", "reply": "Dance mode, watch this."}),
    (r"\bturn around\b|\b180\b", {"intent": "rotate", "degrees": 180, "reply": "Turning around."}),
    (r"\bspin\b", {"intent": "rotate", "degrees": 360, "reply": "Spinning."}),
    (r"\bturn (to the )?left\b", {"intent": "rotate", "degrees": 90, "reply": "Turning left."}),
    (r"\bturn (to the )?right\b", {"intent": "rotate", "degrees": -90, "reply": "Turning right."}),
    (r"\bpomodoro\b", {"intent": "pomodoro", "work_min": 25, "break_min": 5, "reply": "Pomodoro started, focus time."}),
]


MOVE_RE = re.compile(r"\b(mov\w*|go(ing)?|drift\w*|slid\w*|shift\w*|scoot\w*|float\w*|fly(ing)?|nudg\w*|step\w*|come|back up)\b.*?\b(left|right|forwards?|ahead|back(wards?)?)\b")
MOVE_WORD = {"forwards": "forward", "ahead": "forward", "backward": "back", "backwards": "back"}


def move_intent(t):
    """"would you mind moving to the left for me slightly" -> nudge left. A direction after a verb of motion is not nuance:
    it acts at any sentence length ("turn left" is a rotation and has no such verb)."""
    if re.search(r"\bback (up|off|away)\b", t): return {"intent": "nudge", "move": "back", "metres": 0.5, "reply": "Backing up."}
    m = MOVE_RE.search(t)
    if not m or re.search(r"\b(turn|rotate|spin)\b", t): return None
    w = m.group(4); w = MOVE_WORD.get(w, w)
    far = 1.0 if re.search(r"\b(lot|far|more|metre|meter)\b", t) else 0.5
    return {"intent": "nudge", "move": w, "metres": far, "reply": f"Moving {w}."}


def fast_intent(text):
    t0 = " " + re.sub(r"[^a-z0-9 ]", " ", text.lower()) + " "
    mv = move_intent(t0)
    if mv is not None and not re.search(r"\b(don t|do not|stop|never)\b", t0): return mv
    if len(text.split()) > 6:                         # long sentences carry nuance -> model
        return None
    t = " " + re.sub(r"[^a-z0-9 ]", " ", text.lower()) + " "
    if re.search(r"\b(timer|remind|minutes?|seconds?)\b", t) and not re.search(r"\bpomodoro\b", t):
        return None                                    # numbers involved -> let the model parse them
    for pat, it in FAST:
        if re.search(pat, t):
            return dict(it)
    return None


def parse_intent(text, model=MODEL):
    fast = fast_intent(text)
    if fast is not None:
        return fast
    msgs = [{"role": "system", "content": SYSTEM}]
    for u, a in FEW_SHOT:
        msgs += [{"role": "user", "content": u}, {"role": "assistant", "content": json.dumps({**_NULLS, **a})}]
    msgs.append({"role": "user", "content": text})
    r = ollama.chat(model=model, messages=msgs, format=SCHEMA, options={"temperature": 0.1, "num_predict": 160}, keep_alive=-1)
    try:
        d = json.loads(r["message"]["content"])
    except ValueError:
        d = {"intent": "none", "reply": "Sorry, I didn't catch that."}
    d = {k: v for k, v in d.items() if v is not None}          # back to the sparse shape behaviors.py expects
    d.setdefault("reply", "Okay.")
    return d


def warmup(model=MODEL):
    """Load the model into GPU memory so the first real request is fast."""
    ollama.chat(model=model, messages=[{"role": "user", "content": "hi"}], options={"num_predict": 1}, keep_alive=-1)


if __name__ == "__main__":
    import time
    if len(sys.argv) > 1:
        t0 = time.time(); print(parse_intent(" ".join(sys.argv[1:])), f"({time.time() - t0:.1f}s)")
    else:
        warmup()
        while True:
            s = input("you> ").strip()
            if not s: break
            t0 = time.time(); print("  ", parse_intent(s), f"({time.time() - t0:.1f}s)")
