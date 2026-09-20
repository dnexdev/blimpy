"""The room as Blimpy should understand it: who is where RELATIVE TO BLIMPY, in the world frame the floor mat defines.

The room camera (laptop/vision/mono.py + people.py) knows every person's label and floor position; the pilot knows
where the balloon is and which way it faces. This module joins the two, so that "am I on your left?" is answered from
Blimpy's position and heading, not from which side of the webcam picture somebody is on, and so that a voice command
can name a person ("follow P2", "go behind Raymond", "my friend").

Pure functions + one small state holder (Scene). No sockets, no camera. tools/scene_test.py drives it.
  scene = Scene();  scene.update(meta)            # meta: the X-Blimpy-Meta of laptop/vision/eyes.py
  scene.caption(est_p, psi, psi_ok)               # one line for the strip under Blimpy's picture
  label, why_not = scene.resolve("other")         # "me" | "other" | "P2" | a name  ->  a label, or a truthful sentence
  img = scene.annotate(frame, est_p, psi, psi_ok) # boxes + labels + the BLIMPY marker + the caption strip
"""
import math
import numpy as np
from .. import config

P = config.PEOPLE
SECTORS = ("AHEAD", "AHEAD-LEFT", "LEFT", "BEHIND-LEFT", "BEHIND", "BEHIND-RIGHT", "RIGHT", "AHEAD-RIGHT")


def rel(xy, balloon_xy, psi):
    """(ahead m, left m, distance m) of world point xy seen from the balloon at balloon_xy facing psi (rad, world frame,
    counter-clockwise +). + left matches behaviors.NUDGE_DIRS and the mixer's yaw sign."""
    dx, dy = xy[0] - balloon_xy[0], xy[1] - balloon_xy[1]
    c, s = math.cos(psi), math.sin(psi)
    return dx * c + dy * s, -dx * s + dy * c, math.hypot(dx, dy)


def sector(ahead, left):
    """One of eight words for a direction relative to Blimpy's nose."""
    a = math.degrees(math.atan2(left, ahead)) % 360.0            # 0 = ahead, 90 = left
    return SECTORS[int(((a + 22.5) % 360.0) // 45.0)]


def picture_side(box, width):
    cx = (box[0] + box[2]) / 2.0 / max(1.0, width)
    return "left of the picture" if cx < 0.38 else "right of the picture" if cx > 0.62 else "middle of the picture"


def where_words(person, balloon_xy, psi, psi_ok):
    """How to say where one person is. Exact positions in metres relative to Blimpy; approximate ones hedged; unknown
    ones fall back to the picture, and SAY that it is the picture."""
    xyz, q = person.get("xyz"), person.get("q")
    if xyz is None or balloon_xy is None:
        return "close to the camera, position unknown" if xyz is None else f"at ({xyz[0]:.1f}, {xyz[1]:.1f}) m on the floor map"
    ahead, left, d = rel(xyz, balloon_xy, psi if psi is not None else 0.0)
    dist = f"{'about ' if q != 'feet' else ''}{d:.1f} m"
    if psi is None or not psi_ok: return f"{dist} from Blimpy (Blimpy's heading not known yet)"
    return f"{dist} {sector(ahead, left)} of Blimpy"


class Scene:
    """What the pilot knows about the people in the room: the newest roster from mono, the names bound to labels by
    voice, and whom Blimpy is acting on. Names die with mono's run (labels restart at P1) and with an `amb` flag."""

    def __init__(self, params=None):
        self.P = dict(P, **(params or {}))
        self.people, self.meta, self.run = [], {}, None
        self.names, self.bound_t = {}, {}   # label -> name, and mono's frame time when it was bound
        self.target = None               # label Blimpy ACTS on (follows, goes to). NOT "who is speaking": see resolve("me")
        self.dropped = []                # (label, name) dropped because a re-attachment was a close call: for the log

    def update(self, meta):
        """meta None = the room camera is not answering: nobody is known to be anywhere (never the last roster for ever)."""
        if not meta:
            self.people, self.meta = [], {"lost": "no room camera"}; return
        if meta.get("run") != self.run:
            self.run, self.names, self.bound_t, self.target = meta.get("run"), {}, {}, None
        self.meta, self.people = meta, list(meta.get("people") or [])
        for p in self.people:                               # a close call NEWER than the binding: that name may be on the wrong person
            if p["id"] in self.names and p.get("amb", 0) > self.bound_t.get(p["id"], 0):
                self.dropped.append((p["id"], self.names.pop(p["id"]))); self.bound_t.pop(p["id"], None)

    # ------------------------------------------------------------------ who
    def labels(self): return [p["id"] for p in self.people]
    def get(self, label): return next((p for p in self.people if p["id"] == label), None)
    def title(self, label): return f"{label} {self.names[label]}" if label in self.names else label

    def nearest_mic(self):
        """The label most likely to be the SPEAKER when nothing better is known: the microphone sits at the camera, so
        the person nearest the camera. Someone so close that head and feet are out of the picture has no metres: they
        are the nearest by construction (the largest such box wins)."""
        cam = self.meta.get("cam")
        best, key = None, None
        for p in self.people:
            if p.get("near"):                               # head and feet out of the picture: standing at the camera
                b = p["box"]; k = (0, -(b[2] - b[0]) * (b[3] - b[1]))
            elif p.get("xyz") is None: return None          # somebody's distance is unknown: they might be the nearest, so no guess
            elif cam is not None:
                k = (1, math.hypot(p["xyz"][0] - cam[0], p["xyz"][1] - cam[1]))
            else: continue
            if key is None or k < key: best, key = p["id"], k
        return best

    def bind(self, label, name):
        name = " ".join(str(name).encode("ascii", "ignore").decode().split())[:24]      # the marks are drawn with an ASCII-only font
        if not name: return False
        for k in [k for k, v in self.names.items() if v.lower() == name.lower()]: del self.names[k]; self.bound_t.pop(k, None)   # one person per name
        self.names[label] = name; self.bound_t[label] = int(self.meta.get("t") or 0)
        return True

    def resolve(self, who):
        """-> (label, None) or (None, a truthful sentence for the model to say).
        "me" is whoever is SPEAKING, and nothing here can hear who that is: the best guess, made afresh every time, is the
        person nearest the microphone (it sits at the camera). It is never the person Blimpy happens to be following:
        after "follow my friend", "me" is still me."""
        seen = self.labels(); who = " ".join(str(who or "me").split())
        listing = ", ".join(self.title(k) for k in seen) or "nobody"
        if not seen: return None, "I can't see anybody on my room camera right now."
        low = who.lower()
        if low in ("me", "speaker", "i", "myself", "us"):
            k = seen[0] if len(seen) == 1 else self.nearest_mic()
            return (k, None) if k else (None, f"I see {listing}, but I can't tell which of you is speaking. Tell me the label.")
        if low in ("other", "friend", "the other", "the other one", "my friend", "them", "him", "her"):
            me = self.nearest_mic() if len(seen) > 1 else seen[0]
            rest = [k for k in seen if k != me]
            if len(rest) == 1 and me is not None: return rest[0], None
            return None, (f"I only see {listing}." if len(seen) == 1 else f"I see {listing}. Tell me which one: say the label.")
        first = low.split()[0]
        if first[:1] == "p" and first[1:].isdigit():            # "P2", "p2", "P2 Raymond" (as drawn on the picture)
            k = "P" + first[1:]
            return (k, None) if k in seen else (None, f"I don't see {k} right now. I see {listing}.")
        hits = [k for k, v in self.names.items() if v.lower() == low] or [k for k, v in self.names.items() if low in v.lower().split() or v.lower().startswith(low)]
        if len(hits) == 1:
            k = hits[0]
            return (k, None) if k in seen else (None, f"I don't see {self.names[k]} right now. I see {listing}.")
        return None, f"I don't know who {who} is. I see {listing}. Tell me a label, or tell me who is who."

    def behind_point(self, label, balloon_xy, arena=None, margin=0.0):
        """A floor point BEHIND_M beyond that person on the line from Blimpy (so 'go behind Raymond' ends with Raymond
        between Blimpy's start and its goal). None when the person has no EXACT position, the balloon has none, or that
        point is outside the room (arena ((x0, y0), (x1, y1)) shrunk by margin): there is no room behind them."""
        p = self.get(label)
        if p is None or p.get("xyz") is None or p.get("q") != "feet" or balloon_xy is None: return None
        dx, dy = p["xyz"][0] - balloon_xy[0], p["xyz"][1] - balloon_xy[1]; d = math.hypot(dx, dy)
        if d < 1e-6: return None
        x, y = p["xyz"][0] + dx / d * self.P["BEHIND_M"], p["xyz"][1] + dy / d * self.P["BEHIND_M"]
        if arena is not None:
            (x0, y0), (x1, y1) = arena
            if not (x0 + margin <= x <= x1 - margin and y0 + margin <= y <= y1 - margin): return None
        return (x, y)

    def near(self, balloon_xy, range_m):
        """How many people with a known position are within range_m of the balloon; None when that cannot be known
        (no balloon fix, a stale camera pose, or somebody in view whose position is unknown: they might be the one)."""
        if balloon_xy is None or self.meta.get("lost"): return None
        known = [p for p in self.people if p.get("xyz") is not None]
        if len(known) < len(self.people): return None
        return sum(1 for p in known if math.hypot(p["xyz"][0] - balloon_xy[0], p["xyz"][1] - balloon_xy[1]) < range_m)

    # ------------------------------------------------------------------ words and marks
    def caption(self, balloon_xy, psi, psi_ok):
        if self.meta.get("lost"): return "room camera: positions unknown right now"
        if not self.people: return "nobody in view"
        parts = []
        if len(self.people) > 1 and (k := self.nearest_mic()): parts.append(f"nearest the mic: {k}")     # first: the strip may be cut short
        parts += [f"{self.title(p['id'])}: {where_words(p, balloon_xy, psi, psi_ok)}" for p in self.people]
        if self.meta.get("nominal"): parts.append("distances approximate")
        return " | ".join(parts)

    def snapshot(self, balloon_xy, psi, psi_ok):
        """For the session log: what the room looked like when somebody spoke (the data a speaker cue would be measured on)."""
        return {"people": [dict(id=p["id"], q=p.get("q"), xyz=p.get("xyz"), box=p.get("box"), name=self.names.get(p["id"])) for p in self.people],
                "target": self.target, "nearest_mic": self.nearest_mic(), "balloon": None if balloon_xy is None else [round(float(v), 2) for v in balloon_xy[:2]],
                "psi": None if psi is None else round(float(psi), 3), "psi_ok": bool(psi_ok), "lost": self.meta.get("lost")}

    def annotate(self, img, balloon_xyz, psi, psi_ok, meta=None):
        """A COPY of the picture with everything Blimpy should be able to point at: each person's box and label (name
        when bound), the balloon marked BLIMPY with its heading, the mat's axes, and the caption strip. Drawn at full
        size so the labels survive the 640 px downscale on the way to the model."""
        import cv2
        if meta is not None and meta is not self.meta:      # draw the people that came WITH this picture, not the main loop's newer roster
            view = Scene(self.P); view.run, view.names, view.bound_t, view.target = meta.get("run"), self.names, self.bound_t, self.target
            view.meta, view.people = meta, list(meta.get("people") or [])
            return view.annotate(img, balloon_xyz, psi, psi_ok)
        out = img.copy(); h, w = out.shape[:2]; s = w / 1280.0
        font = cv2.FONT_HERSHEY_SIMPLEX
        def tag(text, x, y, color, scale=0.9):
            (tw, th), base = cv2.getTextSize(text, font, scale * s, max(2, int(2 * s)))
            x = int(min(max(2, x), w - tw - 6)); y = int(min(max(th + 6, y), h - 6))
            cv2.rectangle(out, (x - 3, y - th - 5), (x + tw + 3, y + base + 1), (0, 0, 0), -1)
            cv2.putText(out, text, (x, y), font, scale * s, color, max(2, int(2 * s)), cv2.LINE_AA)
        Pm = self.meta.get("P")
        def project(X):
            if Pm is None: return None
            v = np.float64(Pm) @ np.float64([X[0], X[1], X[2], 1.0])
            return None if v[2] <= 1e-6 else (float(v[0] / v[2]), float(v[1] / v[2]))
        if Pm is not None and not self.meta.get("lost"):                       # the tag mat: origin and 1 m axes on the floor
            o = project((0, 0, 0))
            for axis, name in (((1, 0, 0), "+X 1 m"), ((0, 1, 0), "+Y 1 m")):
                e = project(axis)
                if o and e:
                    cv2.arrowedLine(out, (int(o[0]), int(o[1])), (int(e[0]), int(e[1])), (255, 200, 0), max(2, int(2 * s)), cv2.LINE_AA, tipLength=0.08)
                    tag(name, e[0] + 4, e[1], (255, 200, 0), 0.55)
        for p in self.people:
            x1, y1, x2, y2 = [int(v) for v in p["box"]]
            color = (0, 255, 255) if p["id"] == self.target else (0, 255, 0)
            cv2.rectangle(out, (x1, y1), (x2, y2), color, max(2, int(3 * s)))
            tag(self.title(p["id"]), x1 + 4, y1 + int(34 * s), color, 1.1)
        if balloon_xyz is not None and not self.meta.get("lost"):
            b = project(balloon_xyz)
            if b is not None and 0 <= b[0] < w and 0 <= b[1] < h:
                cv2.drawMarker(out, (int(b[0]), int(b[1])), (255, 0, 255), cv2.MARKER_CROSS, int(34 * s), max(2, int(3 * s)))
                tag("BLIMPY (you)", b[0] + 12, b[1] - 12, (255, 0, 255), 1.0)
                if psi is not None and psi_ok:
                    n = project((balloon_xyz[0] + 0.6 * math.cos(psi), balloon_xyz[1] + 0.6 * math.sin(psi), balloon_xyz[2]))
                    if n is not None:
                        cv2.arrowedLine(out, (int(b[0]), int(b[1])), (int(n[0]), int(n[1])), (255, 0, 255), max(2, int(3 * s)), cv2.LINE_AA, tipLength=0.3)
                        tag("nose", n[0] + 6, n[1], (255, 0, 255), 0.6)
        cap = self.caption(None if balloon_xyz is None else balloon_xyz[:2], psi, psi_ok)
        lines, line = [], ""
        for part in cap.split(" | "):
            if len(line) + len(part) > 78 and line: lines.append(line); line = part
            else: line = (line + " | " + part) if line else part
        lines.append(line); lines = lines[:6]
        bar = int((12 + 30 * len(lines)) * s)
        strip = np.zeros((bar, w, 3), np.uint8)
        for i, t in enumerate(lines):
            cv2.putText(strip, t, (int(10 * s), int((28 + 30 * i) * s)), font, 0.78 * s, (255, 255, 255), max(1, int(2 * s)), cv2.LINE_AA)
        return np.vstack([out, strip])
