"""Offline checks of the pilot's world model of people (laptop/control/scene.py) and the picture server (laptop/vision/eyes.py). ~1 s.

  python tools/scene_test.py

 1. rel / sector: eight directions relative to BLIMPY's nose (+ left), whatever the webcam's view is
 2. where_words: exact = metres, approximate = "about", no heading = distance only and says so, no position = says so
 3. resolve: me (nearest the mic, then sticky), other / my friend (only when it is unambiguous), a label, a bound name,
    and truthful sentences when it cannot
 4. names die with mono's run and with an amb flag; one person per name
 5. behind_point: 1.5 m beyond the person on the line from Blimpy; near(): people around the balloon, None when unknown
 6. caption + annotate: the strip names everybody, the picture grows by the strip, the frame is not modified in place
 7. eyes: server round trip (JPEG + meta in one reply), ?target reaches mono and lapses, no server = (None, None)
Run it after touching laptop/control/scene.py, laptop/vision/eyes.py or the people message."""
import math, os, pathlib, sys, time
ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT)); os.chdir(ROOT)
import cv2, numpy as np
from laptop.control import scene as sc
from laptop.vision import eyes as ey

results = {}
def check(name, ok, detail=""):
    results[name] = bool(ok); print(f"  {'PASS' if ok else 'FAIL'}  {name}  {detail}")

# 1. directions: Blimpy at the origin facing +X
want = {(2, 0): "AHEAD", (2, 2): "AHEAD-LEFT", (0, 2): "LEFT", (-2, 2): "BEHIND-LEFT", (-2, 0): "BEHIND", (-2, -2): "BEHIND-RIGHT", (0, -2): "RIGHT", (2, -2): "AHEAD-RIGHT"}
got = {xy: sc.sector(*sc.rel(xy, (0, 0), 0.0)[:2]) for xy in want}
check("eight sectors relative to Blimpy's nose (+ = left)", got == want, str(got))
a, l, d = sc.rel((1.0, 1.0), (1.0, -1.0), math.pi / 2)                      # facing +Y, the point is straight ahead
check("rel() turns with the heading", abs(a - 2.0) < 1e-9 and abs(l) < 1e-9 and abs(d - 2.0) < 1e-9, f"{a:.2f} {l:.2f} {d:.2f}")

# 2. words
P1 = {"id": "P1", "xyz": [2.0, 1.0, 1.1], "q": "feet", "box": [700, 200, 800, 560]}
P2 = {"id": "P2", "xyz": [0.5, -1.5, 1.1], "q": "head", "box": [300, 150, 520, 719]}
P3 = {"id": "P3", "xyz": None, "q": None, "box": [900, 0, 1279, 719], "near": 1}      # head and feet out of the picture: at the laptop
w1, w2, w3 = sc.where_words(P1, (0, 0), 0.0, True), sc.where_words(P2, (0, 0), 0.0, True), sc.where_words(P3, (0, 0), 0.0, True)
check("exact = metres + direction; approximate = 'about'; no position = says so", w1 == "2.2 m AHEAD-LEFT of Blimpy" and w2.startswith("about 1.6 m") and "RIGHT" in w2
      and "position unknown" in w3, f"{w1} | {w2} | {w3}")
check("no confident heading = distance only, and it says why (never a guessed side)", "heading not known" in sc.where_words(P1, (0, 0), 0.3, False)
      and "LEFT" not in sc.where_words(P1, (0, 0), None, False))

# 3. resolve
S = sc.Scene(); meta = {"run": 1, "lost": None, "cam": [-2.3, 0.8, 0.7], "people": [P1, P2], "P": None, "size": [1280, 720]}
S.update(meta)
check("me = the person nearest the microphone (the mic is at the camera)", S.nearest_mic() == "P2" and S.resolve("me") == ("P2", None), str(S.resolve("me")))
S.update(dict(meta, people=[P1, P2, P3]))
check("... someone so close that head and feet are out of the picture is the nearest", S.nearest_mic() == "P3")
S.update(meta); S.target = "P1"                                             # Blimpy is following the FRIEND (P1); the speaker is still P2
check("after 'follow my friend', 'me' is still me: the speaker is never the person being followed", S.resolve("me") == ("P2", None))
check("other / my friend = the other one when there are exactly two", S.resolve("my friend") == ("P1", None) and S.resolve("other") == ("P1", None))
S.update(dict(meta, people=[P1, dict(P2, xyz=None, q=None)]))
check("nobody's distance from the mic known: 'me' is refused with the labels, not guessed", S.resolve("me")[0] is None and "which of you" in S.resolve("me")[1], str(S.resolve("me")))
S.update(dict(meta, people=[P1])); check("one person in view: 'me' is that person, 'other' says there is only one", S.resolve("me") == ("P1", None) and S.resolve("other")[0] is None)
S.update(dict(meta, people=[P1, P2, P3])); lab, why = S.resolve("other")
check("... and a question back when there are three", lab is None and "which one" in why, why)
S.update(dict(meta, people=[P1, P2, P3]))
check("a label in view resolves; one that is not says who IS in view", S.resolve("p2") == ("P2", None) and S.resolve("P7")[0] is None and "P1" in S.resolve("P7")[1], S.resolve("P7")[1])
S.bind("P2", "  Peter "); S.bind("P1", "Raymond")
check("a bound name resolves, any case", S.resolve("peter") == ("P2", None) and S.resolve("RAYMOND") == ("P1", None) and S.title("P2") == "P2 Peter")
check("... so does the label exactly as it is drawn on the picture, and the start of a name", S.resolve("P2 Peter") == ("P2", None) and S.resolve("ray") == ("P1", None))
check("a name the ASCII font cannot draw is reduced to what it can (or refused when nothing is left)", S.bind("P2", "Zo\u00eb") and S.names["P2"] == "Zo" and S.bind("P2", "\u738b") is False)
S.bind("P2", "Peter")
lab, why = S.resolve("Zoe"); check("an unknown name gets a truthful sentence with the roster in it", lab is None and "Zoe" in why and "P1 Raymond" in why, why)
S.update(dict(meta, people=[])); check("nobody in view says so", S.resolve("me")[0] is None and "anybody" in S.resolve("me")[1])

# 4. names die with the run and with an amb flag
S.update(meta); S.bind("P2", "Peter"); S.bind("P1", "Peter")
check("one person per name: re-binding moves it", S.names == {"P1": "Peter"}, str(S.names))
S.update(dict(meta, t=5000, people=[dict(P1, amb=4000), P2])); S.bind("P1", "Peter")                  # bound at t=5000, AFTER the close call at 4000
S.update(dict(meta, t=5100, people=[dict(P1, amb=4000), P2]))
check("a name bound AFTER a close call is kept (the flag is a time, not a curse)", S.names.get("P1") == "Peter", str(S.names))
S.update(dict(meta, t=9000, people=[dict(P1, amb=8000), P2]))
check("a NEWER close call drops the name on that label", "P1" not in S.names and S.dropped[-1] == ("P1", "Peter"))
S.update(None); check("the room camera stops answering: nobody is known to be anywhere (not the last roster for ever)", S.people == [] and S.resolve("me")[0] is None and S.near((0, 0), 3) is None)
S.bind("P2", "Peter"); S.target = "P2"; S.update(dict(meta, run=2))
check("mono restarted (labels restart): names and the target are forgotten", S.names == {} and S.target is None)

# 5. geometry helpers
S.update(meta); bp = S.behind_point("P1", (0.0, 0.0)); d1 = math.hypot(2.0, 1.0)
check("behind = BEHIND_M beyond the person on the line from Blimpy", abs(math.hypot(*bp) - (d1 + sc.P["BEHIND_M"])) < 1e-6 and abs(bp[1] / bp[0] - 0.5) < 1e-6 and sc.P["BEHIND_M"] >= 1.4, str(bp))
check("behind a person without an EXACT position is refused; so is a point outside the room", S.behind_point("P9", (0, 0)) is None and S.behind_point("P2", (0, 0)) is None
      and S.behind_point("P1", (0, 0), arena=((-2, -2), (3, 2)), margin=0.5) is None and S.behind_point("P1", (0, 0), arena=((-2, -2), (6, 4)), margin=0.5) is not None)
S.update(meta)
check("near(): people with a position within range of the balloon", S.near((1.8, 0.8), 1.0) == 1 and S.near((9, 9), 1.0) == 0)
S.update(dict(meta, people=[P1, P3])); check("... None (unknown), not a count, when somebody in view has no position: they might be the one", S.near((9, 9), 1.0) is None)
S.update(dict(meta, lost="drift")); check("... and None while the camera pose is lost", S.near((0, 0), 3.0) is None and "positions unknown" in S.caption((0, 0), 0.0, True))

# 6. caption + annotate
S.update(dict(meta, nominal=True, P=[[900, 0, 640, 100], [0, 900, 360, 200], [0, 0, 1, 3]])); S.bind("P1", "Raymond"); S.target = "P1"
cap = S.caption((0, 0), 0.0, True)
check("the strip leads with the mic guess, names everybody and where they are from Blimpy, and warns 'approximate'", cap.startswith("nearest the mic: P2") and "P1 Raymond: 2.2 m AHEAD-LEFT of Blimpy" in cap
      and "P2: about" in cap and "approximate" in cap, cap)
other = {"t": 1, "run": 1, "lost": None, "people": [dict(P1, box=[10, 10, 60, 200])], "P": None, "size": [1280, 720]}
o2 = S.annotate(img_pre := np.full((720, 1280, 3), 90, np.uint8), None, None, False, meta=other)
check("annotate(meta=...) draws the people that came WITH that picture, not the newer roster", (o2[10:200, 8:14] != 90).any() and (o2[200:560, 798:803] == 90).all())
img = np.full((720, 1280, 3), 90, np.uint8); before = img.copy()
out = S.annotate(img, (0.5, 0.2, 1.5), 0.3, True)
check("annotate: a COPY, as wide as the picture, taller by the strip, something drawn", (img == before).all() and out.shape[1] == 1280 and out.shape[0] > 720 and (out[:720] != before).any(), str(out.shape))
snap = S.snapshot((0, 0), 0.0, True)
check("snapshot for the session log: people, names, target, mic guess", snap["target"] == "P1" and snap["nearest_mic"] == "P2" and snap["people"][0]["name"] == "Raymond")

# 7. eyes
PORT = 5919
srv = ey.EyesServer(PORT).start(); re_ = ey.RoomEyes(PORT, hz=20)
check("no picture yet -> (None, None), no exception", re_.poll() is False and re_.latest() == (None, None))
ok, jpg = cv2.imencode(".jpg", img); from laptop.control.protocol import now_ms
T = now_ms(); srv.publish(jpg.tobytes(), {"t": T, "run": 7, "people": [P1], "lost": None})
re_.target = "P2"
check("round trip: the picture and ITS people arrive together", re_.poll() and re_.latest()[0].shape == (720, 1280, 3) and re_.latest()[1]["t"] == T and re_.latest()[1]["people"][0]["id"] == "P1")
check("?target reaches mono", srv.target == "P2")
re_.target = None; re_.poll(); check("target none clears it", srv.target is None)
srv._t_target = time.monotonic() - 10; srv._target = "P1"; check("a target lapses when the pilot stops asking", srv.target is None)
srv.stop(); dead = ey.RoomEyes(PORT + 1, hz=20)
try: r = dead.poll()
except Exception: r = None
check("no server: poll fails quietly, latest() is (None, None)", not r and dead.latest() == (None, None))
re_._meta["t"] = T - 10000; check("a live server with a FROZEN camera (old frame time) is not served as the present", re_.latest() == (None, None))
re_._meta["t"] = now_ms(); re_._t_ok -= 10; check("no answer for a while: not served either", re_.latest() == (None, None))

n_fail = sum(1 for v in results.values() if not v)
print(f"\n{len(results) - n_fail}/{len(results)} checks passed")
sys.exit(1 if n_fail else 0)
