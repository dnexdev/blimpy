"""Venue geometry: the room the balloon flies in, as DATA (venues/<name>.json) instead of constants.

  python -m laptop.positioning.venue                      # print + validate venues/default.json
  python -m laptop.positioning.venue venues/other.json

World frame = PROTOCOL.md s5 (origin at the floor AprilTag, +X tag left->right, +Y tag bottom->top, +Z up, metres).
laptop/config.py loads the file at import and exports ARENA / OBSTACLES / JUDGES_XY / WANDER_BOX in the shapes every
consumer already expects (nested 2-tuples, list of (x, y, r) tuples, 2-tuple or None), so nothing downstream knows
about this module. Add or move a place with  python -m laptop.positioning.capture_place <name>.
Switch rooms with the BLIMPY_VENUE environment variable (path to another json).

Stdlib only on purpose: config.py imports this, so it must stay cheap and must not import anything from laptop.
"""
import json, math, os, sys
from dataclasses import dataclass, field
from pathlib import Path

DEFAULT_PATH = Path(__file__).resolve().parents[2] / "venues" / "default.json"   # repo root, not the cwd
EPS = 1e-6


@dataclass
class Obstacle:
    """Vertical cylinder the balloon must not touch: table, pillar, tripod, person-sized things."""
    x: float
    y: float
    r: float
    name: str = ""

    def as_tuple(self):
        return (float(self.x), float(self.y), float(self.r))


@dataclass
class Venue:
    name: str = "unnamed"
    arena: tuple = ((-2.0, -2.0), (2.0, 2.0))     # ((min_x, min_y), (max_x, max_y)): the walls
    obstacles: list = field(default_factory=list)   # [Obstacle]
    places: dict = field(default_factory=dict)      # name -> (x, y): "judges", "home", "charger", ...
    wander: tuple | None = None                     # ((min_x, min_y), (max_x, max_y)) or None = arena shrunk by the controller
    origin: str = ""                                # free text: where the floor tag is, which way +X points
    notes: str = ""
    version: int = 1

    # --- shapes the rest of the code expects (laptop/config.py re-exports these) ---
    def arena_tuple(self):
        (x0, y0), (x1, y1) = self.arena
        return ((float(x0), float(y0)), (float(x1), float(y1)))

    def obstacle_tuples(self):
        return [o.as_tuple() for o in self.obstacles]

    def place(self, name):
        p = self.places.get(name)
        return None if p is None else (float(p[0]), float(p[1]))

    def wander_tuple(self):
        if self.wander is None:
            return None
        (x0, y0), (x1, y1) = self.wander
        return ((float(x0), float(y0)), (float(x1), float(y1)))

    def obstacle(self, name):
        return next((o for o in self.obstacles if o.name == name), None)

    def to_json(self):
        return {
            "name": self.name, "version": self.version, "units": "m", "origin": self.origin,
            "arena": {"min": list(self.arena[0]), "max": list(self.arena[1])},
            "obstacles": [{"name": o.name, "x": o.x, "y": o.y, "r": o.r} for o in self.obstacles],
            "places": {k: [float(v[0]), float(v[1])] for k, v in self.places.items()},
            "wander": None if self.wander is None else {"min": list(self.wander[0]), "max": list(self.wander[1])},
            "notes": self.notes,
        }


def _box(d, what):
    """{"min": [x, y], "max": [x, y]} -> ((x, y), (x, y)), with a clear error if the shape is off."""
    try:
        (x0, y0), (x1, y1) = d["min"], d["max"]
        return ((float(x0), float(y0)), (float(x1), float(y1)))
    except (KeyError, TypeError, ValueError) as e:
        raise ValueError(f"venue {what}: expected {{\"min\": [x, y], \"max\": [x, y]}}, got {d!r}") from e


def load(path=DEFAULT_PATH):
    """Parse a venue json. Raises FileNotFoundError / ValueError loudly: the file is committed, a typo should be
    visible on every script start. Does NOT validate geometry (that is validate(), run by the tests and tools)."""
    path = Path(path)
    if not path.exists():
        raise FileNotFoundError(f"venue file {path} missing (venues/default.json is committed; BLIMPY_VENUE={os.environ.get('BLIMPY_VENUE')!r})")
    try:
        d = json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as e:
        raise ValueError(f"venue file {path}: invalid JSON at line {e.lineno}: {e.msg}") from e
    if not isinstance(d, dict) or "arena" not in d:
        raise ValueError(f"venue file {path}: top level must be an object with at least 'arena'")
    obstacles = []
    for i, o in enumerate(d.get("obstacles") or []):
        try:
            obstacles.append(Obstacle(float(o["x"]), float(o["y"]), float(o["r"]), str(o.get("name", ""))))
        except (KeyError, TypeError, ValueError) as e:
            raise ValueError(f"venue file {path}: obstacles[{i}] needs x, y, r (and optional name), got {o!r}") from e
    places = {}
    for k, v in (d.get("places") or {}).items():
        try:
            places[str(k)] = (float(v[0]), float(v[1]))
        except (TypeError, ValueError, IndexError) as e:
            raise ValueError(f"venue file {path}: places[{k!r}] must be [x, y], got {v!r}") from e
    return Venue(name=str(d.get("name", path.stem)), arena=_box(d["arena"], "arena"), obstacles=obstacles, places=places,
                 wander=None if d.get("wander") is None else _box(d["wander"], "wander"),
                 origin=str(d.get("origin", "")), notes=str(d.get("notes", "")), version=int(d.get("version", 1)))


def save(venue, path=DEFAULT_PATH):
    """Write atomically (temp file + os.replace) so a crash mid-write cannot leave a half json behind."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(venue.to_json(), indent=2) + "\n", encoding="utf-8")
    os.replace(tmp, path)
    return path


def _inside(x, y, box, shrink=0.0):
    (x0, y0), (x1, y1) = box
    return x0 + shrink - EPS <= x <= x1 - shrink + EPS and y0 + shrink - EPS <= y <= y1 - shrink + EPS


def validate(venue, r_balloon=0.55, standoff=0.5):
    """Geometry sanity -> (errors, warnings), each a list of strings. Errors = the controller would fly into
    something or the sampling is impossible; warnings = tighter than the README recommends (0.5 m standoff from
    the judges' table edge). Tolerances of EPS so a place exactly at the limit passes."""
    errors, warnings = [], []
    (x0, y0), (x1, y1) = venue.arena
    if not (x0 < x1 and y0 < y1):
        errors.append(f"arena min must be < max on both axes: {venue.arena}")
        return errors, warnings
    names = [o.name for o in venue.obstacles if o.name]
    for n in sorted(set(names)):
        if names.count(n) > 1:
            errors.append(f"obstacle name {n!r} used {names.count(n)} times")
    for o in venue.obstacles:
        tag = o.name or f"({o.x}, {o.y})"
        if o.r <= 0:
            errors.append(f"obstacle {tag}: radius must be > 0, got {o.r}")
        if not _inside(o.x, o.y, venue.arena):
            errors.append(f"obstacle {tag}: centre ({o.x}, {o.y}) is outside the arena")
    for name, (px, py) in venue.places.items():
        if not _inside(px, py, venue.arena, shrink=r_balloon):
            errors.append(f"place {name!r} ({px}, {py}): less than R_BALLOON={r_balloon} m from a wall (or outside)")
        for o in venue.obstacles:
            edge = math.hypot(px - o.x, py - o.y) - o.r
            tag = o.name or f"({o.x}, {o.y})"
            if edge < r_balloon - EPS:
                errors.append(f"place {name!r} overlaps obstacle {tag}: {edge:.2f} m from its edge, need >= {r_balloon}")
            elif edge < r_balloon + standoff - EPS:
                warnings.append(f"place {name!r} is {edge:.2f} m from obstacle {tag}; README asks for >= {r_balloon + standoff:.2f}")
    if venue.wander is not None:
        (wx0, wy0), (wx1, wy1) = venue.wander
        if not (wx0 < wx1 and wy0 < wy1):
            errors.append(f"wander min must be < max on both axes: {venue.wander}")
        elif not (_inside(wx0, wy0, venue.arena) and _inside(wx1, wy1, venue.arena)):
            errors.append(f"wander box {venue.wander} sticks out of the arena")
    return errors, warnings


def describe(venue):
    (x0, y0), (x1, y1) = venue.arena
    lines = [f"venue {venue.name!r}  arena x {x0}..{x1} m, y {y0}..{y1} m  ({x1 - x0:.1f} x {y1 - y0:.1f} m)"]
    if venue.origin:
        lines.append(f"  origin: {venue.origin}")
    for o in venue.obstacles:
        lines.append(f"  obstacle {o.name or '?':14s} at ({o.x:+.2f}, {o.y:+.2f}) r={o.r:.2f}")
    for k, (px, py) in venue.places.items():
        lines.append(f"  place    {k:14s} at ({px:+.2f}, {py:+.2f})")
    lines.append(f"  wander   {venue.wander if venue.wander else 'None (arena shrunk by the controller)'}")
    if venue.notes:
        lines.append(f"  notes: {venue.notes}")
    return "\n".join(lines)


def main():
    path = Path(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT_PATH
    v = load(path)
    print(describe(v))
    errors, warnings = validate(v)
    for w in warnings: print(f"  WARNING {w}")
    for e in errors: print(f"  ERROR   {e}")
    print("OK" if not errors else f"{len(errors)} error(s)")
    sys.exit(0 if not errors else 1)


if __name__ == "__main__":
    main()
