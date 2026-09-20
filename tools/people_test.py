"""Offline checks of the people roster (laptop/vision/people.py) and of mono.step with a roster. No camera, no YOLO, ~1 s.

  python tools/people_test.py

 1. signatures: a white and a black/orange shirt separate; the same shirt under +-20 % brightness does not
 2. locate(): feet in view = exact; feet cut, head in view = head ray at the assumed height; both cut = listed, no metres
 3. two people cross and the tracker SWAPS their ids: the labels stay with the shirts
 4. a person leaves for 5 s and returns under a new tracker id: same label
 5. an ambiguous return (two lost people who look alike) keeps the labels from multiplying but is flagged amb
 6. labels are never reused; a one-frame ghost is never published
 7. the single-person message: sticky primary (the largest box flipping does not change who "the person" is), a primary
    whose feet are cut gives person = null and does NOT jump to the other person, `want` picks a label
 8. mono.step: roster=None is the old behaviour; with a roster everybody is in dets["people"]; 6 people fit a datagram
Run it after touching laptop/vision/people.py or mono.step."""
import json, os, pathlib, sys
ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT)); os.chdir(ROOT)
import numpy as np
from laptop.vision import mono, people
from tools.vision_test import synth_cam, observe

results = {}
def check(name, ok, detail=""):
    results[name] = bool(ok); print(f"  {'PASS' if ok else 'FAIL'}  {name}  {detail}")

W, H = 1280, 720
cam = synth_cam("room", (-2.3, 0.8, 0.9), (1.5, 0.0, 1.15), f=900.0, size=(W, H), dist=(0, 0, 0, 0, 0))
NEAR, AT_LAPTOP = (-0.44, 0.41), (-0.9, 0.55)      # 1.9 m from the camera: feet below the picture, head in it | 1.4 m: both out


def shirt(color, bright=1.0, seed=0):
    """A fake frame region painter: returns a function that paints a person of that shirt colour into an image."""
    rng = np.random.default_rng(seed)
    def paint(img, box):
        x1, y1, x2, y2 = [int(v) for v in box]
        patch = np.clip(np.array(color, np.float32) * bright + rng.normal(0, 6, (max(1, y2 - y1), max(1, x2 - x1), 3)), 0, 255)
        img[max(0, y1):y2, max(0, x1):x2] = patch.astype(np.uint8)[:img[max(0, y1):y2, max(0, x1):x2].shape[0], :img[max(0, y1):y2, max(0, x1):x2].shape[1]]
    return paint

WHITE, BLACK, ORANGE = (235, 235, 235), (25, 25, 28), (30, 120, 240)


def box_of(xy, height=1.75, width=0.5):
    """Pixel box of a person standing at floor point xy (feet on the floor, head at `height`)."""
    f = observe(cam, (xy[0], xy[1], 0.0)); h = observe(cam, (xy[0], xy[1], height))
    hw = abs(f[1] - h[1]) * width / height / 2
    return (f[0] - hw, h[1], f[0] + hw, f[1])


def det(tid, box):
    x1, y1, x2, y2 = box
    e = 4
    return {"id": tid, "box": (max(0.0, x1), max(0.0, y1), min(W, x2), min(H, y2)), "pt": ((x1 + x2) / 2, y1 + 0.35 * (y2 - y1)),
            "cut": {"top": y1 <= e, "bottom": y2 >= H - e, "left": x1 <= e, "right": x2 >= W - e}, "area": (x2 - x1) * (y2 - y1)}


def tick(roster, t, items):
    """items: [(tracker id, floor xy, shirt painter)] -> (public list, dets)."""
    img = np.full((H, W, 3), 128, np.uint8); dets = []
    for tid, xy, paint in items:
        d = det(tid, box_of(xy)); paint(img, d["box"]); d["near"] = people.very_near(cam, d); dets.append(d)      # (mono.step sets near)
    dets.sort(key=lambda d: -d["area"])
    out = roster.update(dets, [people.locate(cam, d) for d in dets], [people.signature(img, d["box"]) for d in dets], t)
    return out, dets


# 1. signatures
img = np.full((H, W, 3), 128, np.uint8); b = (500, 200, 640, 640)
def sig(color, bright=1.0, seed=0):
    im = img.copy(); shirt(color, bright, seed)(im, b); return people.signature(im, b)
d_wb, d_wo = people.sig_dist(sig(WHITE), sig(BLACK)), people.sig_dist(sig(WHITE), sig(ORANGE))
d_same = max(people.sig_dist(sig(ORANGE), sig(ORANGE, 0.8, 1)), people.sig_dist(sig(ORANGE), sig(ORANGE, 1.2, 2)), people.sig_dist(sig(BLACK), sig(BLACK, 1.2, 3)))
check("signatures: different shirts far apart, the same shirt under +-20 % light close", d_wb > 0.6 and d_wo > 0.6 and d_same < people.P["SIG_MAX"],
      f"white-black {d_wb:.2f}, white-orange {d_wo:.2f}, same shirt worst {d_same:.2f} (limit {people.P['SIG_MAX']})")
check("signature of a sliver is None, distance with None is None", people.signature(img, (10, 10, 14, 300)) is None and people.sig_dist(None, sig(WHITE)) is None)

# 2. locate
xyz, q = people.locate(cam, det(1, box_of((1.5, 0.2))))
check("locate: feet in view -> exact floor point", q == "feet" and abs(xyz[0] - 1.5) < 0.05 and abs(xyz[1] - 0.2) < 0.05, f"{xyz} {q}")
near = det(2, box_of(NEAR))
xyz2, q2 = people.locate(cam, near)
check("locate: feet cut, head in view -> head ray at the assumed height, flagged", near["cut"]["bottom"] and not near["cut"]["top"] and q2 == "head"
      and abs(xyz2[0] - NEAR[0]) < 0.35 and abs(xyz2[1] - NEAR[1]) < 0.35, f"{xyz2} {q2} (true {NEAR})")
lap = det(5, box_of(AT_LAPTOP))
check("locate: someone right at the laptop (head and feet out) -> no metres", lap["cut"]["bottom"] and lap["cut"]["top"] and people.locate(cam, lap) == (None, None))
both = det(3, (400, 0, 640, H))
check("locate: head AND feet out of the picture -> listed without metres", people.locate(cam, both) == (None, None))
side = det(4, (W - 120, 200, W, 600))
check("locate: cut by the side of the picture -> position flagged approx", people.locate(cam, side)[1] == "approx", str(people.locate(cam, side)))
check("locate: fakes without a cut key still work", people.locate(cam, {"id": 9, "box": box_of((1.5, 0.2)), "pt": (0, 0)})[1] == "feet")

# 3. crossing with a tracker id swap
R = people.Roster(); A, B = shirt(WHITE, seed=1), shirt(BLACK, seed=2)
for k in range(6): out, _ = tick(R, 100 * k, [(11, (1.6, 0.6), A), (12, (1.6, -0.6), B)])
lab = {p["id"]: p["xyz"][1] for p in out}
pa = [k for k, y in lab.items() if y > 0][0]; pb = [k for k, y in lab.items() if y < 0][0]
for k in range(6, 12): out, _ = tick(R, 100 * k, [(12, (1.6, 0.5), A), (11, (1.6, -0.5), B)])      # ids swapped by the tracker
lab2 = {p["id"]: p["xyz"][1] for p in out}
check("two people cross, tracker ids swap: labels stay with the shirts", lab2.get(pa, 0) > 0 and lab2.get(pb, 0) < 0 and len(R.people) == 2, f"before {lab} after {lab2}")

# 4. leave and return under a new tracker id
for k in range(12, 16): tick(R, 100 * k, [(12, (1.6, 0.5), A)])                                     # B walks out
out, _ = tick(R, 1500 + 5000, [(12, (1.6, 0.5), A), (31, (2.4, -1.0), B)])
for k in range(3): out, _ = tick(R, 6600 + 100 * k, [(12, (1.6, 0.5), A), (31, (2.4, -1.0), B)])
check("a person gone 5 s returns under a new tracker id: same label", {p["id"] for p in out} == {pa, pb}, str([p["id"] for p in out]))

# 5. ambiguous return -> new label
R2 = people.Roster(); G1, G2 = shirt((90, 90, 95), seed=5), shirt((92, 90, 93), seed=6)
for k in range(5): tick(R2, 100 * k, [(1, (1.5, 0.7), G1), (2, (1.5, -0.7), G2)])
for k in range(3): out, _ = tick(R2, 10000 + 100 * k, [(7, (2.5, 0.0), G1)])                         # both lost 9.5 s, look alike
check("an ambiguous return: no new label (labels must not multiply), flagged amb so the name on it is dropped", len(out) == 1 and out[0]["id"] in ("P1", "P2")
      and out[0].get("amb") == 10000 and len(R2.people) == 2, str(out))
check("... the flag is the TIME of the close call (a name bound after it can be kept, laptop/control/scene.py)", R2.public()[0]["amb"] == 10000)
check("a clear return is NOT flagged", all("amb" not in p for p in R.public()), str(R.public()))
# a box seen once (a duplicate on the same person, the balloon's bag) must not rival real people for two minutes
R6 = people.Roster()
for k in range(5): tick(R6, 100 * k, [(1, (1.6, 0.4), A)])
tick(R6, 500, [(1, (1.6, 0.4), A), (8, (1.7, 0.45), A)])                                              # one frame with a duplicate box
ghost = [k for k, q in R6.people.items() if q.hits < people.P["MIN_HITS"]]
for k in range(6, 20): out, _ = tick(R6, 100 * k, [(1, (1.6, 0.4), A)])
out, _ = tick(R6, 2100, [(42, (1.6, 0.4), A)])                                                        # the tracker loses and re-finds the person
check("a one-frame duplicate box is forgotten after GHOST_S and never makes the real person's return a close call",
      len(ghost) == 1 and ghost[0] not in R6.people and [p["id"] for p in out] == ["P1"] and "amb" not in out[0], f"{list(R6.people)} {out}")
# a person the tracker follows without a break is never handed the label of somebody who left
R7 = people.Roster()
for k in range(5): tick(R7, 100 * k, [(1, (1.6, 0.5), A), (2, (2.2, -0.6), B)])
for k in range(5, 40): tick(R7, 100 * k, [(1, (1.6, 0.5), A)])                                        # B left 3.5 s ago
out, _ = tick(R7, 4000, [(1, (1.6, 0.5), B)])                                                         # A turns round: a black back, same track
check("a continuously tracked person who shows a new side keeps the label (not handed to someone who left)", [p["id"] for p in out] == ["P1"], str(out))
# a view cut by the frame is a view too
R8 = people.Roster()
for k in range(6): tick(R8, 100 * k, [(1, (1.6, 0.4), A)])
n0 = len(R8.people[1].sigs)
half = shirt(ORANGE, seed=11)
for k in range(6, 10): tick(R8, 100 * k, [(1, AT_LAPTOP, half)])                                      # walks up to the laptop: cut top and bottom, looks different
check("a new view is learned while the tracker vouches for it, even when the box is cut by the frame", len(R8.people[1].sigs) > n0 and R8.public()[0].get("near") == 1, f"{n0} -> {len(R8.people[1].sigs)} {R8.public()}")
# the feet-cut tier refuses an ill-conditioned ray (a head at the camera's own height)
seated = det(6, (560, 352, 700, 719))
check("feet cut and the head at the camera's height: no metres (the ray runs along the assumed-height plane)", people.locate(cam, seated) == (None, None), str(people.locate(cam, seated)))

# 6. labels never reused; ghosts never published
R3 = people.Roster({"FORGET_S": 1.0})
for k in range(4): tick(R3, 100 * k, [(1, (1.5, 0.5), A)])
out_ghost, _ = tick(R3, 400, [(1, (1.5, 0.5), A), (5, (2.5, -0.8), shirt(ORANGE, seed=7))])
for k in range(4): out, _ = tick(R3, 5000 + 100 * k, [(9, (2.0, 0.0), shirt(ORANGE, seed=8))])
check("a one-frame ghost is not published; a forgotten label is never reused", [p["id"] for p in out_ghost] == ["P1"] and out[0]["id"] == "P3" and 1 not in R3.people,
      f"{[p['id'] for p in out_ghost]} then {[p['id'] for p in out]}")

# 7. the single-person message
R4 = people.Roster()
for k in range(5): tick(R4, 100 * k, [(1, (1.4, 0.5), A), (2, (2.6, -0.6), B)])                       # A nearer = larger
first = R4.select().label
for k in range(5, 10): tick(R4, 100 * k, [(1, (2.8, 0.5), A), (2, (1.3, -0.6), B)])                   # now B is the larger box
check("sticky primary: the largest box flipping does not change who 'the person' is", R4.select().label == first == "P1", f"{first} -> {R4.select().label}")
check("want = a label picks that person; an unknown label picks nobody", R4.select("P2").label == "P2" and R4.select("P9") is None)
for k in range(10, 13): out, _ = tick(R4, 100 * k, [(1, NEAR, A), (2, (2.6, -0.6), B)])             # the primary walks toward the laptop: feet cut
who = R4.select()
check("a primary whose feet are cut is still the primary (the caller holds): it does NOT become the other person", who.label == "P1" and who.q == "head", f"{who.label} {who.q}")
for k in range(13, 16): tick(R4, 100 * k, [(2, (2.6, -0.6), B)])                                      # the primary leaves the picture
check("just out of view: nobody for PRIMARY_HOLD_S, then the person in view takes over", R4.select() is None)
for k in range(3): tick(R4, 1600 + 3000 + 100 * k, [(2, (2.6, -0.6), B)])
check("... and after the hold the person in view becomes the primary", R4.select().label == "P2")

# 8. mono.step
class FakePeople:
    def __init__(self, items): self.items = items
    def detect(self, frame):
        ds = []
        for tid, xy, paint in self.items:
            d = det(tid, box_of(xy)); paint(frame, d["box"]); ds.append(d)
        return sorted(ds, key=lambda d: -d["area"])
class NoBalloon:
    def detect(self, frame): return None
fp = FakePeople([(1, AT_LAPTOP, A), (2, (2.2, -0.5), B)])                                            # the speaker at the laptop is cut top and bottom, the friend is whole
img8 = np.full((H, W, 3), 128, np.uint8)
msg_old, _ = mono.step(cam, fp, NoBalloon(), (img8.copy(), 1000), {"t": None}, 1000)
check("roster=None: the old behaviour, largest box cut -> person null (the bug this fixes stays reproducible)", msg_old["person"] is None and msg_old["person_id"] == -1)
R5 = people.Roster(); prev = {"t": None}
for k in range(4): msg, dets = mono.step(cam, fp, NoBalloon(), (img8.copy(), 1000 + 100 * k), prev, 1000 + 100 * k, roster=R5)
ids = {p["id"]: p for p in dets["people"]}
check("with a roster BOTH people are listed: the cut speaker (no metres invented) and the friend (exact)", set(ids) == {"P1", "P2"} and ids["P1"]["xyz"] is None
      and ids["P2"]["q"] == "feet" and abs(ids["P2"]["xyz"][0] - 2.2) < 0.06, json.dumps(dets["people"]))
check("... 'person' holds (null) while the primary's feet are cut, and says why", msg["person"] is None and "holding" in dets["why"]["person"], dets["why"]["person"])
msg, dets = mono.step(cam, fp, NoBalloon(), (img8.copy(), 1500), prev, 1500, roster=R5, want="P2")
check("want='P2': the single-person message follows the friend, person_id is the label number", msg["person"] is not None and abs(msg["person"][0] - 2.2) < 0.06 and msg["person_id"] == 2 and dets["who"] == "P2", str(msg))
six = [{"id": f"P{i}", "xyz": [-12.345, 10.123, 1.234], "q": "approx", "box": [1000, 100, 1279, 719]} for i in range(1, 7)]
n = len(json.dumps({"t": 123456789, "src": "people", "run": 1234567890123, "lost": None, "who": "P6", "people": six}))
check("six people fit a 2048-byte datagram with room to spare", n < 1200 and people.P["MAX_PUBLISHED"] <= 6, f"{n} bytes")

n_fail = sum(1 for v in results.values() if not v)
print(f"\n{len(results) - n_fail}/{len(results)} checks passed")
sys.exit(1 if n_fail else 0)
