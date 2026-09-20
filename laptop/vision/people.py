"""Everybody in the room camera's view, not just the largest box: a roster of people with labels that stay put.

The detector's tracker gives each box an id, but that id dies when a person is hidden or leaves, and two people who
cross can swap theirs. The roster turns it into a label a human (and the voice model) can use: P1, P2 ... Each person
keeps an APPEARANCE SIGNATURE (a colour histogram of the torso) and a world position from the mat-calibrated camera, so
a returning person gets the old label back and a swapped tracker id is put right. A person is remembered by a small
GALLERY of signatures (front and back of a shirt are different pictures). When a return is ambiguous (two remembered
people look alike) the best match still gets the label, so labels do not multiply, but it is flagged `amb`: the pilot
then drops the NAME bound to that label, because a wrong "that is Raymond" is worse than "someone".

Where a person is on the floor (`locate`), in three honest tiers:
  feet    feet in the picture: bottom-centre of the box meets the floor (mono.person_from_box). Exact (5-10 % of range).
  approx  feet in the picture but the box is cut by a SIDE edge (half a body): same maths, flagged.
  head    feet cut off, head in the picture: the head ray meets the plane z = ASSUMED_H_M. Approximate (people differ).
  None    head and feet both out of the picture (someone standing right at the laptop): listed, no metres invented.

Pure: no sockets, no camera, no clock of its own. tools/people_test.py drives it with fake boxes.
  roster = Roster();  people = roster.update(dets, xyzs, sigs, t_ms);  who = roster.select(want, t_ms)
"""
import dataclasses, math
import cv2, numpy as np
from .. import config

P = config.PEOPLE


def signature(img, box):
    """Normalised HSV histogram (H12 x S4 x V4 = 192 numbers; V is kept: a white and a black shirt differ only there) of
    the clothes: 15-85 % of the box height, the centre 60 % of its width (shirt AND trousers: seen on the recorded
    two-person frames, a torso-only band slid between shirt and trousers whenever the head was cut off, and one person
    read as three). None when the crop is too small to mean much."""
    if img is None: return None
    h, w = img.shape[:2]
    x1, y1, x2, y2 = box; bw, bh = x2 - x1, y2 - y1
    xa, xb = int(max(0, x1 + 0.2 * bw)), int(min(w, x2 - 0.2 * bw))
    ya, yb = int(max(0, y1 + 0.15 * bh)), int(min(h, y1 + 0.85 * bh))
    if xb - xa < 8 or yb - ya < 8: return None
    hsv = cv2.cvtColor(np.ascontiguousarray(img[ya:yb, xa:xb]), cv2.COLOR_BGR2HSV)
    hsv[..., 0][(hsv[..., 1] < 40) | (hsv[..., 2] < 40)] = 0          # grey, white, black: the hue is noise, not a colour
    hist = cv2.calcHist([hsv], [0, 1, 2], None, [12, 4, 4], [0, 180, 0, 256, 0, 256]).astype(np.float32)
    k = np.float32([0.25, 0.5, 0.25])                                  # soft bins: a cloud passing (+-20 % light) must not
    for ax in range(3):                                                # move a shirt into a different histogram
        pad = "wrap" if ax == 0 else "edge"
        hist = np.apply_along_axis(lambda v: np.convolve(np.pad(v, 1, mode=pad), k, mode="valid"), ax, hist)
    hist = hist.ravel(); s = float(hist.sum())
    return (hist / s).astype(np.float32) if s > 0 else None


def sig_dist(a, b):
    """Bhattacharyya distance 0 (same) .. 1 (nothing in common); None when either side has no signature."""
    if a is None or b is None: return None
    return float(math.sqrt(max(0.0, 1.0 - float(np.sum(np.sqrt(a * b))))))


def locate(cam, det, params=P):
    """-> (xyz or None, q) for one detection dict (see the module docstring). Fakes without a "cut" key count as uncut."""
    from . import mono                                   # lazy: mono imports this module
    box = det["box"]; cx = (box[0] + box[2]) / 2.0
    w, h = cam.size; m = params["EDGE_FRAC"]                 # a box that ends within 2 % of the frame edge is cut there: the detector
    cut = dict(det.get("cut") or {})                        # stops a few pixels short of the edge (seen on the recorded frames: legs
    cut["top"] = cut.get("top") or box[1] <= m * h          # cut at 713 of 720 px read as "feet", 1.6 m off)
    cut["bottom"] = cut.get("bottom") or box[3] >= (1 - m) * h
    cut["left"] = cut.get("left") or box[0] <= m * w
    cut["right"] = cut.get("right") or box[2] >= (1 - m) * w
    if not cut.get("bottom"):
        X, _ = mono.person_from_box(cam, box)
        if X is not None:
            return [round(float(v), 3) for v in X], ("approx" if cut.get("left") or cut.get("right") else "feet")
        return None, None
    if not cut.get("top") and not (cut.get("left") or cut.get("right")):
        H = params["ASSUMED_H_M"]
        o, d = mono.ray(cam, (cx, box[1]))
        if abs(d[2]) / max(1e-9, math.hypot(d[0], d[1])) >= params["HEAD_RAY_MIN"]:      # a head near the camera's own height gives a ray
            X = mono.hit_plane(cam, (cx, box[1]), H)                                      # almost parallel to that plane: metres would be noise
            if X is not None and math.hypot(X[0] - o[0], X[1] - o[1]) <= params["HEAD_RANGE_MAX_M"]:
                return [round(float(X[0]), 3), round(float(X[1]), 3), round(mono.CHEST_FRAC * H, 3)], "head"
    return None, None


def very_near(cam, det, params=P):
    """Head AND feet out of the picture: the person stands right at the camera (and the microphone beside it)."""
    h = cam.size[1]; m = params["EDGE_FRAC"]; b = det["box"]
    return b[1] <= m * h and b[3] >= (1 - m) * h


def _iou(a, b):
    x1, y1, x2, y2 = max(a[0], b[0]), max(a[1], b[1]), min(a[2], b[2]), min(a[3], b[3])
    inter = max(0.0, x2 - x1) * max(0.0, y2 - y1)
    ua = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / ua if ua > 0 else 0.0


@dataclasses.dataclass
class Person:
    pid: int
    tid: int
    t_first: int
    t_seen: int
    box: tuple
    xyz: list | None = None
    q: str | None = None
    sigs: list = dataclasses.field(default_factory=list)      # gallery: a few signatures (front, back, in shadow)
    amb: int = 0                                              # t_ms of the last re-attachment that was a close call (0 = never): a name
                                                              # bound BEFORE that moment is not to be trusted, one bound after it is
    near: bool = False                                        # head and feet out of the picture: right at the camera
    hits: int = 1
    area: float = 0.0
    cut: dict = dataclasses.field(default_factory=dict)

    @property
    def label(self): return f"P{self.pid}"


class Roster:
    """Stable labels over the detector's tracker ids. update() once per processed frame."""

    def __init__(self, params=None):
        self.P = dict(P, **(params or {}))
        self.people = {}                 # pid -> Person (seen within FORGET_S)
        self.next_pid = 1
        self.primary = None              # pid of the sticky "the person" for the legacy single-person message
        self.t = 0
        self.visible = []                # pids seen in the last update, largest box first

    # ------------------------------------------------------------------ matching
    def _cost(self, det, xyz, sig, p, t_ms):
        """(cost, by_tid) or None when this detection cannot be that person. A tracker-id match is always allowed (the
        tracker's motion model vouches for it); anything else needs the signature AND, when both positions are known, a
        floor distance the person could have walked in the gap."""
        by_tid = det.get("id", -1) >= 0 and det.get("id") == p.tid and t_ms - p.t_seen < 1000 * self.P["TID_TRUST_S"]
        d_sig = self._dist(sig, p)
        d_pos = None
        if xyz is not None and p.xyz is not None:
            d_pos = math.hypot(xyz[0] - p.xyz[0], xyz[1] - p.xyz[1])
        if not by_tid:
            if d_sig is None or d_sig > self.P["SIG_MAX"]: return None
            gap = max(0.0, (t_ms - p.t_seen) / 1000.0)
            if d_pos is not None and gap < self.P["POS_GATE_S"] and d_pos > self.P["GATE_M"] + self.P["V_MAX"] * gap: return None
        cost = (d_sig if d_sig is not None else self.P["SIG_MAX"]) - (self.P["TID_BONUS"] if by_tid else 0.0)
        if d_pos is not None: cost += 0.1 * min(1.0, d_pos / self.P["GATE_M"])
        return cost, by_tid

    def update(self, dets, xyzs, sigs, t_ms):
        """dets: PersonTracker.detect() dicts; xyzs: [(xyz|None, q)] from locate(); sigs: signature() per det.
        Returns the public list (see public())."""
        self.t = t_ms
        for pid in [k for k, p in self.people.items() if t_ms - p.t_seen > 1000 * self.P["FORGET_S"]]:
            del self.people[pid]
            if self.primary == pid: self.primary = None
        for pid in [k for k, p in self.people.items() if p.hits < self.P["MIN_HITS"] and t_ms - p.t_seen > 1000 * self.P["GHOST_S"]]:
            del self.people[pid]                            # a box seen once or twice (a duplicate, the balloon's bag) is not a person to remember
        live = {p.tid for p in self.people.values() if p.tid >= 0 and t_ms - p.t_seen < 1000 * self.P["TID_TRUST_S"]}
        cand = []                                           # (cost, det index, pid, by_tid)
        for i, d in enumerate(dets):
            tracked = d.get("id", -1) in live               # the tracker says: this box continues somebody we saw a moment ago
            for pid, p in self.people.items():
                c = self._cost(d, xyzs[i][0], sigs[i], p, t_ms)
                if c is None: continue
                if not c[1]:
                    if p.hits < self.P["MIN_HITS"]: continue                                  # never re-attach to an unconfirmed ghost
                    if tracked and t_ms - p.t_seen >= 1000 * self.P["TID_TRUST_S"]: continue    # a continuing track may be SWAPPED with someone
                cand.append((c[0], i, pid, c[1]))                                             # also in view, never handed to someone absent
        cand.sort(key=lambda c: c[0])
        det_pid, used, close_call = {}, set(), set()
        for cost, i, pid, by_tid in cand:
            if i in det_pid or pid in used: continue
            if not by_tid:                                  # a re-attachment that is NOT clearly the best one on offer is flagged
                rivals = [c for c, j, q, b in cand if j == i and q != pid and q not in used and not b]
                if rivals and min(rivals) - cost < self.P["AMBIG_MARGIN"]: close_call.add(pid)
            det_pid[i] = pid; used.add(pid)
        boxes = [d["box"] for d in dets]
        self.visible = []
        for i, d in enumerate(dets):
            xyz, q = xyzs[i]
            cut = d.get("cut") or {}
            clean = not any(cut.values()) and all(_iou(d["box"], b) < 0.1 for j, b in enumerate(boxes) if j != i)
            pid = det_pid.get(i)
            if pid is None:
                pid = self.next_pid; self.next_pid += 1
                self.people[pid] = Person(pid, d.get("id", -1), t_ms, t_ms, tuple(d["box"]), xyz, q,
                                          [sigs[i]] if sigs[i] is not None else [], 0, bool(d.get("near")), 1, d.get("area", 0.0), cut)
            else:
                p = self.people[pid]
                same_track = d.get("id", -1) >= 0 and d.get("id") == p.tid
                p.tid, p.t_seen, p.box, p.hits, p.area, p.cut = d.get("id", -1), t_ms, tuple(d["box"]), p.hits + 1, d.get("area", 0.0), cut
                p.xyz, p.q, p.near = xyz, q, bool(d.get("near"))
                if pid in close_call: p.amb = int(t_ms)
                self._learn(p, sigs[i], clean, same_track, alone=all(_iou(d["box"], b) < 0.1 for j, b in enumerate(boxes) if j != i))
            d["pid"] = self.people[pid].label               # for the debug window and the marks
            self.visible.append(pid)
        self.visible.sort(key=lambda k: -self.people[k].area)
        return self.public()

    def _dist(self, sig, p):
        ds = [d for d in (sig_dist(sig, s) for s in p.sigs) if d is not None]
        return min(ds) if ds else None

    def _learn(self, p, sig, clean, same_track, alone=True):
        """Gallery upkeep. The nearest remembered signature follows the light slowly (clean boxes only). A NEW view of the
        same person (their back, a jacket taken off, or the half of them that fits the picture when they stand at the
        laptop) is added only while the tracker id vouches that it IS that person and nobody overlaps the box."""
        if sig is None: return
        if not p.sigs: p.sigs.append(sig); return
        ds = [sig_dist(sig, s) for s in p.sigs]; k = int(np.argmin(ds))
        if same_track and alone and ds[k] > self.P["GALLERY_NEW"]:
            p.sigs.append(sig); del p.sigs[:-self.P["GALLERY"]]
        elif clean:
            s = (1 - self.P["SIG_EMA"]) * p.sigs[k] + self.P["SIG_EMA"] * sig; p.sigs[k] = (s / float(s.sum())).astype(np.float32)

    # ------------------------------------------------------------------ outputs
    def public(self):
        """What goes on the wire: people in THIS frame seen at least MIN_HITS times, largest first, MAX_PUBLISHED at most:
        [{"id": "P1", "xyz": [x, y, z] | None, "q": "feet" | "approx" | "head" | None, "box": [x1, y1, x2, y2], "amb": t_ms?, "near": 1?}].
        amb = when (mono's frame clock, the same as "t") the label was last re-attached on a close call; near = head and
        feet out of the picture, i.e. standing at the camera."""
        out = []
        for pid in self.visible:
            p = self.people[pid]
            if p.hits < self.P["MIN_HITS"]: continue
            e = {"id": p.label, "xyz": p.xyz, "q": p.q, "box": [int(round(v)) for v in p.box]}
            if p.amb: e["amb"] = p.amb
            if p.near: e["near"] = 1
            out.append(e)
        return out[:self.P["MAX_PUBLISHED"]]

    def select(self, want=None):
        """Who is "the person" of the single-person message (follow_me, behaviors, the simulator)? `want` = a label the
        pilot chose ("P2"): that person or nobody. Otherwise the STICKY primary: whoever it was stays it while in view,
        and for PRIMARY_HOLD_S after leaving; only then the largest box in view takes over. Returns a Person or None.
        The caller publishes a position only for q == "feet": a cut primary means "hold", never "follow someone else"."""
        vis = [k for k in self.visible if self.people[k].hits >= self.P["MIN_HITS"]]
        if want:
            pid = int(want[1:]) if isinstance(want, str) and want[:1] in "Pp" and want[1:].isdigit() else None
            return self.people[pid] if pid in vis else None
        if self.primary in vis: return self.people[self.primary]
        p = self.people.get(self.primary)
        if p is not None and self.t - p.t_seen < 1000 * self.P["PRIMARY_HOLD_S"]: return None      # just out of view: wait for them
        self.primary = vis[0] if vis else None
        return self.people[self.primary] if self.primary is not None else None
