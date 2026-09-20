"""Who is that for? Decides whether a spoken turn was meant for Blimpy, without a wake phrase.

Three layers, each replaceable:
  1. CUES: small independent observers. Each looks at the turn's context (Ctx) and returns evidence in [0, 1] with a
     reason: how much a word sounds like the name, how fresh the conversation with Blimpy is, whether Blimpy had just
     asked something, whether someone is in front of its eye, how the sentence is built. A new signal (speaker direction,
     gaze, a voice print) is one more class in CUES; nothing else changes.
  2. ROOM: the evidence is weighed against two thresholds that slide with how loud the room is (0 = the private judging
     room, 1 = the science-fair floor): at or above `accept` the turn is for Blimpy, at or below `reject` it is not. A loud
     room raises the bar, shortens the memory of the conversation and stops trusting the eye.
  3. JUDGE: what falls BETWEEN the thresholds is a question of meaning, not of word lists: a language model reads the
     sentence with the conversation around it ("P. How are you feeling?" is the name clipped by the transcriber; "wait,
     where's the other?" right after Blimpy spoke is not for Blimpy). Backends: the relay's text model, a local ollama
     model, or none (the midpoint of the thresholds decides). Clear cases never reach the judge: no cost, no delay.

Every number lives in DEFAULTS and can be overridden from config.OMNI["ADDRESSEE"].

  python -m laptop.voice.addressee "yeah recording started" --since-reply 2           # the cues, the score, the verdict
  python -m laptop.voice.addressee "P. How are you feeling?" --judge relay
"""
import dataclasses, difflib, json, math, os, re, time

DEFAULTS = dict(
    names=("blimpy",),                       # config NAME_WORDS: the name and the spellings the transcriber has produced
    weights=dict(name=1.0, summon=1.0, engaged=0.5, form=0.4, answer=0.35, presence=0.3, judge=0.5),
    name_sim=(0.75, 0.90),                   # similarity to the name: no evidence below, full evidence above
    engaged_tau_s=(8.0, 5.0),                # how fast the conversation with Blimpy fades (quiet, loud room)
    accept=(0.80, 0.80), reject=(0.15, 0.20),   # (quiet, loud). Between them: the judge, or `midpoint` without one. reject is low on
                                             # purpose: a turn with ANY real evidence (still in conversation, built like a command) is a
                                             # question of meaning, and a verdict costs a few hundred text tokens; only evidence-free chatter is free
    midpoint=(0.525, 0.625),                 # no judge (off, failed, too slow): at or above this the turn is for Blimpy
    summon_s=(8.0, 5.0),                     # "Hey, Blimpy." covers the next turn when it starts within this
    judge_frames=2,                          # pictures from the turn shown to the judge (relay only; 0 = words only)
    judge_timeout_s=4.0, history=6,          # measured on the relay: 0.7-1.8 s per verdict, a rare 3 s straggler (seen live: it was the one
                                             # turn that mattered). Slower than this = the midpoint decides
    min_evidence=0.02,                       # a cue contributing less than this says nothing (a conversation that faded a minute ago)
)


@dataclasses.dataclass
class Ctx:
    """Everything known about one turn. None = not known (no transcript yet, no eye)."""
    text: str | None = None
    since_reply_s: float | None = None       # speech started this long after Blimpy's last words to an addressed turn (0 = talked over it)
    since_named_s: float | None = None       # ... after the last turn that used the name
    since_summon_s: float | None = None      # ... after a bare "Hey, Blimpy."
    asked: bool = False                      # Blimpy's last words were a question and this is the first turn since
    presence: bool | None = None             # someone near and centred in Blimpy's eye
    people: int | None = None                # how many are near and centred (None = not known)
    loudness: float = 0.0                    # 0 quiet room .. 1 loud room
    history: tuple = ()                      # ((who, text), ...) oldest first


@dataclasses.dataclass
class Decision:
    ok: bool | None                          # None = between the thresholds: ask the judge
    score: float
    why: str
    parts: dict


def lerp(pair, x): return pair[0] + (pair[1] - pair[0]) * min(1.0, max(0.0, x))


# ---------------------------------------------------------------------------------------------------------- the name
_SOUND = str.maketrans("bdgvzmqcy", "ptkfsnkki")


def sound(word):
    """A crude sound key: voiced/unvoiced pairs merged, vowels merged, doubles dropped ("blimby" == "blimpy")."""
    k = re.sub(r"[aeiou]+", "a", word.lower().replace("ph", "f").translate(_SOUND))
    return re.sub(r"(.)\1+", r"\1", k)


def name_similarity(text, names):
    """0..1: the closest any word (or two neighbours: "blim pea") comes to the name, by spelling and by sound."""
    w = re.findall(r"[a-z]+", (text or "").lower())
    best = 0.0
    for c in w + [a + b for a, b in zip(w, w[1:])]:
        for n in names:
            s = 0.5 * difflib.SequenceMatcher(None, c, n).ratio() + 0.5 * difflib.SequenceMatcher(None, sound(c), sound(n)).ratio()
            best = max(best, s)
    return best


def name_only(text, names, full=0.9):
    """"Hey, Blimpy." and nothing more: a summons. The sentence it announces arrives as the NEXT turn, because the
    server's voice detector cuts at the pause after the name."""
    w = re.findall(r"[a-z']+", (text or "").lower())
    rest = [t for t in w if name_similarity(t, names) < full]
    return 0 < len(w) <= 3 and len(rest) < len(w) and len(rest) <= 1


# ---------------------------------------------------------------------------------------------- how the sentence is built
# The judge decides meaning. This is the cheap stand-in that works with no judge at all: does the sentence have the FORM
# of something said to a robot (an imperative from the vocabulary of Blimpy's own tool, a request, a question to "you")?
FILLERS = ("ok", "okay", "now", "and", "then", "so", "please", "hey", "alright", "right", "um", "uh", "well", "also", "actually", "just", "yeah", "yes")
REQUESTS = (("can", "you"), ("could", "you"), ("would", "you"), ("will", "you"), ("i", "want", "you"), ("i", "need", "you"), ("let's",), ("lets",))
IMPERATIVES = {   # first word -> the words allowed next (None = anything; a bare imperative always counts)
    "follow": None, "stay": None, "spin": None, "rotate": None, "dance": None, "wander": None, "roam": None, "explore": None,
    "hover": None, "land": None, "fly": None, "float": None, "rise": None, "climb": None, "descend": None, "higher": None, "lower": None,
    "go": ("to", "home", "up", "down", "higher", "lower", "left", "right", "back", "forward", "over", "there", "around"),
    "come": ("here", "over", "back", "to", "with", "down", "up", "closer"),
    "turn": ("left", "right", "around", "to", "by", "clockwise", "counter", "counterclockwise", "a"),
    "look": ("at", "here", "over"), "watch": ("me", "this"), "keep": ("me", "an", "watching"), "tell": ("me", "us"), "show": ("me", "us"),
    "set": ("timer", "pomodoro"), "start": ("timer", "pomodoro", "focus"), "cancel": ("timer", "pomodoro", "focus"),   # within the next 4 words
}
QUESTIONS = (r"\b(what|who|where|how|why) (do|can|are|did|would|will|were) you\b", r"\b(do|did|can|could|are|were|will|would|have) you\b",
             r"\bwhat('s| is| are) (this|that|these|those|in my|on my)\b", r"\bwhat am i\b", r"\bhow (much time|long)\b",
             r"\byour (name|battery|job)\b", r"\bwho are you\b")


def directed(text):
    low = (text or "").lower()
    w = re.findall(r"[a-z']+", low)
    while w and w[0] in FILLERS: w = w[1:]
    if not w: return False
    if any(tuple(w[:len(r)]) == r for r in REQUESTS): return True
    if w[0] in ("up", "down", "left", "right") and len(w) <= 3: return True
    if w[0] in IMPERATIVES:
        nxt = IMPERATIVES[w[0]]
        if nxt is None or len(w) == 1: return True
        return any(x in nxt for x in (w[1:5] if w[0] in ("set", "start", "cancel") else w[1:2]))
    return any(re.search(q, low) for q in QUESTIONS)


# ---------------------------------------------------------------------------------------------------------------- cues
class Cue:
    """One observer. evidence(ctx, P) -> (0..1, reason) or None when it has nothing to say. `key` names its weight."""
    key = ""
    def evidence(self, ctx, P): raise NotImplementedError


class NameCue(Cue):
    key = "name"
    def evidence(self, ctx, P):
        if not ctx.text: return None
        lo, hi = P["name_sim"]; s = name_similarity(ctx.text, P["names"])
        return (min(1.0, (s - lo) / (hi - lo)), "name" if s >= hi else f"sounds like the name ({s:.2f})") if s > lo else None


class SummonCue(Cue):
    key = "summon"
    def evidence(self, ctx, P):
        if ctx.since_summon_s is None or not ctx.text or not 0 <= ctx.since_summon_s < lerp(P["summon_s"], ctx.loudness): return None
        return 1.0, "called by name just before"


class EngagedCue(Cue):
    """The conversation with Blimpy fades: full evidence while it talks, 1/e after tau seconds (tau shorter in a loud room)."""
    key = "engaged"
    def evidence(self, ctx, P):
        ages = [a for a in (ctx.since_reply_s, ctx.since_named_s) if a is not None and a >= 0]
        if not ages: return None
        return math.exp(-min(ages) / lerp(P["engaged_tau_s"], ctx.loudness)), f"in conversation ({min(ages):.0f} s)"


PERSONAL = re.compile(r"\b(you|your|i|me|my|we|us)\b")


class FormCue(Cue):
    """Full evidence for the form of a command / request / question to "you". Half for any other QUESTION that involves the
    speaker or the listener ("what colour shirt am I wearing?"): not enough to be accepted, enough to be worth the judge's
    reading in a quiet room (seen live: asked of Blimpy under a mangled name, it scored zero and was dismissed unread)."""
    key = "form"
    def evidence(self, ctx, P):
        if directed(ctx.text): return 1.0, "said like a command or a question to a robot"
        t = (ctx.text or "").lower()
        if t.rstrip().endswith("?") and PERSONAL.search(t): return 0.5, "a question involving the speaker or the listener"
        return None


class AnswerCue(Cue):
    """Blimpy asked; the next thing said is probably the answer. A question back is not an answer."""
    key = "answer"
    def evidence(self, ctx, P):
        if not ctx.asked or (ctx.text or "").rstrip().endswith("?"): return None
        return 1.0, "answer to Blimpy's question"


class PresenceCue(Cue):
    """Said to its face: counts only together with the form of a command, and less the louder (more crowded) the room."""
    key = "presence"
    def evidence(self, ctx, P):
        if not ctx.presence or not directed(ctx.text) or ctx.loudness >= 1.0: return None
        n = max(1, ctx.people or 1)             # a group in front of it: the evidence is shared out, nobody in particular is "its face"
        return (1.0 - ctx.loudness) / n, "said to its face" if n == 1 else f"said in front of it ({n} people)"


CUES = (NameCue(), SummonCue(), EngagedCue(), FormCue(), AnswerCue(), PresenceCue())


# -------------------------------------------------------------------------------------------------------------- decide
class Addressee:
    def __init__(self, params=None, cues=CUES):
        p = dict(DEFAULTS); o = dict(params or {})
        p["weights"] = dict(DEFAULTS["weights"], **o.pop("weights", {}))
        p.update({k: v for k, v in o.items() if v is not None})
        self.P, self.cues = p, tuple(cues)

    def decide(self, ctx):
        parts = {}
        for c in self.cues:
            ev = c.evidence(ctx, self.P)
            if ev and self.P["weights"].get(c.key, 0.0) * ev[0] >= self.P["min_evidence"]: parts[c.key] = (self.P["weights"][c.key] * ev[0], ev[1])
        score = sum(v for v, _ in parts.values())
        top = " + ".join(why for _, why in sorted(parts.values(), reverse=True)) or "no name"
        acc, rej = lerp(self.P["accept"], ctx.loudness), lerp(self.P["reject"], ctx.loudness)
        if score >= acc: return Decision(True, score, top, parts)
        if score <= rej: return Decision(False, score, self._no(ctx, parts), parts)
        return Decision(None, score, top, parts)

    def resolve(self, ctx, dec, p=None, judge_why=""):
        """Settle an undecided turn: with the judge's probability p that it was for Blimpy, or (p None: no judge, it
        failed, it was too slow) by the midpoint of the thresholds."""
        if dec.ok is not None: return dec
        mid = lerp(self.P["midpoint"], ctx.loudness)
        score = dec.score + (self.P["weights"]["judge"] * (2.0 * p - 1.0) if p is not None else 0.0)
        ok = score >= mid
        why = (f"judge: {judge_why or ('for Blimpy' if ok else 'not for Blimpy')}" if p is not None
               else (dec.why if ok else self._no(ctx, dec.parts)))
        return Decision(ok, score, why, dec.parts)

    @staticmethod
    def _no(ctx, parts):
        if "engaged" in parts: return "in conversation, but not said to Blimpy"
        return "no name" + (", loud room" if ctx.loudness >= 0.5 and "form" in parts else "")

    def is_name(self, text): return name_similarity(text, self.P["names"]) >= self.P["name_sim"][1]
    def is_summons(self, text): return name_only(text, self.P["names"], self.P["name_sim"][1])


# --------------------------------------------------------------------------------------------------------------- judges
JUDGE_PROMPT = """A small flying robot called "{name}" shares a room with people who mostly talk to EACH OTHER. It acts on
commands (follow me, turn, go up, stop, timers) and answers questions put to it. A speech recogniser wrote down what was
just said. It cannot spell the robot's name: it clips it or writes a similar-sounding word or name instead ("P.", "Blimby",
"Limpy"), so an odd word or an unfamiliar name used the way one would call somebody is probably the robot's name.
Room: {room}. {facts}
Recent conversation (oldest first):
{history}
Now someone says: "{text}"
Was that said TO the robot? Remarks between people, status updates ("recording started"), questions about objects or
other people, and thinking aloud are not. Answer with one JSON object only:
{{"for_robot": true/false, "confidence": 0.0-1.0, "why": "<= 8 words"}}"""


PICTURES = (" The attached picture(s) were taken in the room WHILE this was said (the camera stands beside the microphone; the robot "
            "may be the balloon in the picture). Someone looking or gesturing at the robot or at the camera speaks for the robot; "
            "people facing each other, a screen or a phone speak against.")


def judge_prompt(ctx, names, pictures=0):
    facts = []
    if ctx.since_reply_s is not None: facts.append(f"The robot last spoke {ctx.since_reply_s:.0f} s before this." if ctx.since_reply_s > 0 else "The speaker talked over the robot.")
    if ctx.asked: facts.append("The robot's last words were a question.")
    if ctx.presence is not None:
        facts.append("Nobody is near the robot." if not ctx.presence else "Someone is standing close to the robot."
                     if (ctx.people or 1) == 1 else f"{ctx.people} people are standing close to the robot.")
    if pictures: facts.append(PICTURES.strip())
    hist = "\n".join(f"  {who}: {t}" for who, t in ctx.history) or "  (nothing yet)"
    return JUDGE_PROMPT.format(name=names[0].capitalize(), room="loud, crowded hall" if ctx.loudness >= 0.5 else "quiet room, a few people",
                               facts=" ".join(facts), history=hist, text=ctx.text)


def parse_judge(txt):
    """-> (probability it was for the robot, why) or (None, error)."""
    m = re.search(r"\{.*\}", txt or "", re.S)
    try: d = json.loads(m.group(0))
    except Exception: return None, f"no json in {(txt or '')[:60]!r}"
    c = min(1.0, max(0.0, float(d.get("confidence", 0.7))))
    return (0.5 + 0.5 * c if d.get("for_robot") else 0.5 - 0.5 * c), str(d.get("why", ""))[:60]


class RelayJudge:
    """The relay's text model over HTTP (chat completions). Every call goes to the usage ledger (purpose addressee_judge)."""
    def __init__(self, key=None, model=None, http=None, names=DEFAULTS["names"], timeout=DEFAULTS["judge_timeout_s"], ledger=True):
        self.key = key or os.environ.get("OMNI_API_KEY") or os.environ.get("YIBU_API_KEY")
        self.model = model or os.environ.get("OMNI_JUDGE_MODEL") or os.environ.get("OMNI_WATCH_MODEL", "qwen3.5-omni-flash")
        self.http = http or os.environ.get("OMNI_HTTP", "https://yibuapi.com/v1")
        self.names, self.timeout, self.ledger = names, timeout, ledger      # ledger: True (the default file) | a path | False

    wants_frames = True

    def ask(self, ctx, frames=()):
        """frames: base64 JPEGs from the turn (what Blimpy saw while it was said)."""
        import requests
        from . import usage_log
        text = judge_prompt(ctx, self.names, len(frames))
        content = text if not frames else [{"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{f}"}} for f in frames] + [{"type": "text", "text": text}]
        body = {"model": self.model, "temperature": 0.0, "max_tokens": 60, "messages": [{"role": "user", "content": content}]}
        t0 = time.monotonic(); usage = None; status = None; p = None; why = ""
        try:
            r = requests.post(f"{self.http}/chat/completions", json=body, timeout=self.timeout,
                              headers={"Authorization": f"Bearer {self.key}", "Content-Type": "application/json"})
            status = r.status_code; d = r.json(); usage = d.get("usage")
            if status != 200: why = f"http {status}: {str(d.get('error', d))[:120]}"
            else: p, why = parse_judge(d["choices"][0]["message"].get("content"))
        except Exception as e:
            why = f"{e.__class__.__name__}: {e}"
        if self.ledger: usage_log.record(self.model, self.key, "addressee_judge", f"{self.http}/chat/completions", "http", p is not None,
                                         (time.monotonic() - t0) * 1000, usage, None if p is not None else why, status_code=status,
                                         path=self.ledger if isinstance(self.ledger, str) else None)
        return p, why


class OllamaJudge:
    """A local model (no key, no internet): the same question to ollama. BLIMPY_LLM picks the model, as for intent.py."""
    def __init__(self, model=None, names=DEFAULTS["names"], **_):
        self.model = model or os.environ.get("BLIMPY_LLM", "qwen2.5:7b"); self.names = names

    def ask(self, ctx):
        try:
            import ollama
            r = ollama.chat(model=self.model, messages=[{"role": "user", "content": judge_prompt(ctx, self.names)}], format="json",
                            options={"temperature": 0.0, "num_predict": 60}, keep_alive=-1)
            return parse_judge(r["message"]["content"])
        except Exception as e:
            return None, f"{e.__class__.__name__}: {e}"


def make_judge(kind, **kw):
    """"relay" | "ollama" | "off"/None, or any object with ask(ctx) -> (p or None, why)."""
    if not kind or kind == "off": return None
    if hasattr(kind, "ask"): return kind
    return {"relay": RelayJudge, "ollama": OllamaJudge}[kind](**kw)


if __name__ == "__main__":
    import argparse
    from .. import load_env                                   # noqa: F401  (.env read on package import)
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("text"); ap.add_argument("--since-reply", type=float); ap.add_argument("--asked", action="store_true")
    ap.add_argument("--presence", action="store_true"); ap.add_argument("--loudness", type=float, default=0.0)
    ap.add_argument("--judge", default="off", choices=("off", "relay", "ollama"))
    a = ap.parse_args()
    A = Addressee(); ctx = Ctx(a.text, since_reply_s=a.since_reply, asked=a.asked, presence=a.presence or None, loudness=a.loudness)
    dec = A.decide(ctx)
    for k, (v, why) in dec.parts.items(): print(f"  {k:9s} +{v:.2f}  {why}")
    print(f"  score {dec.score:.2f}  accept >= {lerp(A.P['accept'], a.loudness):.2f}  reject <= {lerp(A.P['reject'], a.loudness):.2f}  ->  "
          f"{'undecided' if dec.ok is None else dec.ok}")
    if dec.ok is None:
        j = make_judge(a.judge); t0 = time.monotonic(); p, why = j.ask(ctx) if j else (None, "")
        if j: print(f"  judge p={p} ({why}) in {time.monotonic() - t0:.2f} s")
        dec = A.resolve(ctx, dec, p, why)
    print(f"  {'FOR BLIMPY' if dec.ok else 'not for Blimpy'}: {dec.why}")
