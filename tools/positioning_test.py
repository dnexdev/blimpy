"""Offline regression test of laptop/positioning: no cameras, no controller, numpy only (~1 s).

  python tools/positioning_test.py

(1) schema: row shape, fix_from_state.
(2) session: SessionLog round-trip in a temp dir -> load() shapes, NaN rows, truncated last line, session.json, NullLog.
(3) replay: ReplaySource order and monotonic t, pacing on an injected clock, loop offset, make_source parsing.
(4) fuse: Fuser priority, then freshness, then honest nulls, with an injected clock.
(5) venue: venues/default.json validates clean; bad venues error; save/load round-trip; config exports keep their shapes.
(6) udp: UdpStateSource on port 5017 receives 5 datagrams in one poll (protocol.UdpJson.recv_all).
(7) evaluate: synthetic truth + delayed noisy fixes -> rms, recovered latency, dropouts, jumps, link gaps, wall-time join, plot.
Run it after touching laptop/positioning/*, laptop/config.py or venues/*.
"""
import importlib.util, json, math, os, pathlib, sys, tempfile, time
ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT)); os.chdir(ROOT)
import numpy as np
from laptop import config
from laptop.positioning import evaluate, schema, session, venue
from laptop.positioning.fuse import Fuser
from laptop.positioning.sources import ListSource, ReplaySource, UdpStateSource, make_source
from laptop.control.protocol import UdpJson


def checker(tag):
    results = []

    def check(name, cond, extra=""):
        results.append(bool(cond)); print(f"[{tag}] {name}{'  ' + extra if extra else ''}  {'OK' if cond else 'FAIL'}")
    return results, check


def state(t, balloon, person=None, pid=-1, src="sim"):
    return {"t": t, "balloon": balloon, "person": person, "person_id": pid, "src": src}


def write_session(root, n=20, dt=66):
    """n state rows + n telem rows, recorder clock 5000 + i*dt (telem 10 ms after each state)."""
    log = session.SessionLog(root=root, name="x", tag="test", meta={"k": 1})
    for i in range(n):
        log.write("state", state(1000 + i * dt, None if i == 1 else [i * 0.1, 0.0, 1.7], [0.5, 0.2, 1.3] if i else None, 4 if i else -1), t_ms=5000 + i * dt)
        log.write("telem", {"t": 7 + i, "yaw": 0.1 * i, "yr": 0.0, "alt": 1.6, "armed": 1}, t_ms=5000 + i * dt + 10)
    return log


def write_eval_session(root, seconds=40.0, lag_ms=120, noise=0.02, drop_frac=0.05, seed=0, t0=5000, static=False,
                       outlier=True, tag="fake", wall0=1.7e9, t_shift=0, with_truth=True):
    """Truth every 20 ms on a smooth path (max ~0.75 m/s), state every 66 ms recorded lag_ms after capture with
    gaussian noise, a dropout burst (drop_frac of the run), one 0.6 m outlier row, telemetry at 20 Hz with a 700 ms
    hole, commands at 15 Hz. wall = wall0 + capture time so two sessions with different recorder clocks can be joined."""
    rng = np.random.default_rng(seed)
    log = session.SessionLog(root=root, name="ev", tag=tag)

    def truth_at(s):
        if static:
            return np.array([1.0, 0.5, 1.7]), np.array([2.0, 0.5, 1.3])
        return (np.array([1.5 * math.sin(0.5 * s), 0.8 * math.cos(0.35 * s), 1.7 + 0.05 * math.sin(0.2 * s)]),
                np.array([2.0 + 0.02 * s, 0.5, 1.3]))
    rows = []
    if with_truth:
        for s_ in np.arange(0.0, seconds, 0.02):
            b, pr = truth_at(s_); tm = t0 + s_ * 1000
            rows.append((tm, "truth", {"t": int(tm), "x": b[0], "y": b[1], "z": b[2], "psi": 0.0, "person": pr.tolist(), "armed": True}, wall0 + s_))
    i = 0
    for s_ in np.arange(0.0, seconds, 0.066):
        b, pr = truth_at(s_); cap = t0 + s_ * 1000; rec = cap + lag_ms
        drop = 10.0 <= s_ < 10.0 + drop_frac * seconds
        bal = None if drop else (b + rng.normal(0, noise, 3)).round(4).tolist()
        if outlier and i == 100 and bal is not None:
            bal = [bal[0] + 0.6, bal[1], bal[2]]
        per = (pr + rng.normal(0, noise, 3)).round(4).tolist()
        rows.append((rec, "state", {"t": int(cap), "balloon": bal, "person": per, "person_id": 1, "src": "sim"}, wall0 + rec / 1000 - t0 / 1000))
        i += 1
    for s_ in np.arange(0.0, seconds, 0.05):
        if 20.0 <= s_ < 20.7: continue
        tm = t0 + s_ * 1000
        rows.append((tm, "telem", {"t": int(tm), "yaw": 0.1, "yr": 0.0, "alt": 1.1, "armed": 1, "age": 30}, wall0 + s_))
    for s_ in np.arange(0.0, seconds, 1 / 15):
        tm = t0 + s_ * 1000
        rows.append((tm, "cmd", {"t": int(tm), "vf": 0.2, "vs": 0.0, "yr": 0.0, "vz": 0.0, "arm": 1}, wall0 + s_))
    rows.sort(key=lambda r: r[0])
    for tm, kind, data, wall in rows:
        log.write(kind, data, t_ms=int(tm + t_shift), wall=wall)
    log.event("arm", armed=True)
    log.close()
    return log.path


# ---------- (1) ----------
def test_schema():
    results, check = checker("schema")
    r = schema.row("state", {"t": 5}, t_ms=100, wall=1.5)
    check("row -> {kind, t_ms, wall, data}, data verbatim", set(r) == {"kind", "t_ms", "wall", "data"} and r["data"] == {"t": 5}
          and r["t_ms"] == 100 and r["wall"] == 1.5 and json.dumps(r))
    r = schema.row("cmd", {"vf": 0.1})
    check("row defaults: t_ms and wall filled from the recorder clocks", isinstance(r["t_ms"], int) and r["wall"] > 1e9)
    try:
        schema.row("bogus", {}); bad = False
    except ValueError:
        bad = True
    check("unknown kind -> ValueError", bad)
    f = schema.fix_from_state(state(1, [1, 2, 3], src="vision"))
    check("fix_from_state -> Fix(t, xyz, source)", f.t_ms == 1 and f.xyz == (1.0, 2.0, 3.0) and f.source == "vision" and f.seen)
    check("fix_from_state on null -> None", schema.fix_from_state(state(1, None)) is None
          and schema.fix_from_state(state(1, [0, 0, 0]), "person") is None)
    return all(results)


# ---------- (2) ----------
def test_session():
    results, check = checker("session")
    with tempfile.TemporaryDirectory() as d:
        log = write_session(d, n=3)
        log.cmd({"t": 9, "vf": 0.2, "vs": 0.0, "yr": 0.1, "vz": 0.0, "arm": 1})
        log.truth({"t": 1000, "x": 1.0, "y": 2.0, "z": 1.7, "psi": 0.3, "person": (0.5, 0.2, 1.3), "motors": (0.1, 0.1, 0.0, 0.0), "armed": True})
        log.event("arm", armed=True)
        p = log.path
        check("directory <root>/<ts>_<tag>_<name>", p.parent == pathlib.Path(d) and p.name.endswith("_test_x") and len(p.name) == len("20260913_120000_test_x"))
        check("rows counted", log.n == 9)
        log.close()
        meta = json.loads((p / "session.json").read_text())
        check("session.json: schema, tag, git, config snapshot, meta, rows, ended", meta["schema"] == schema.SCHEMA_VERSION and meta["tag"] == "test"
              and "git" in meta and meta["config"]["ARENA"] == [list(a) for a in config.ARENA] and meta["meta"] == {"k": 1}
              and meta["rows"] == 9 and "ended_wall" in meta, f"git={meta['git']}")
        check("iter_rows yields every row in order", [r["kind"] for r in session.iter_rows(p)] == ["state", "telem"] * 3 + ["cmd", "truth", "event"])
        out = session.load(p)
        b, pe = out["state"]["balloon"], out["state"]["person"]
        check("load: state balloon (N,3) float with a NaN row where null", b.shape == (3, 3) and np.isnan(b[1]).all() and np.isfinite(b[0]).all() and b[2, 0] == 0.2)
        check("load: person NaN row at i=0, values after", np.isnan(pe[0]).all() and pe[1].tolist() == [0.5, 0.2, 1.3])
        check("load: t = recorder clock, t_src = sender clock, wall present", out["state"]["t"].tolist() == [5000, 5066, 5132]
              and out["state"]["t_src"].tolist() == [1000, 1066, 1132] and out["state"]["wall"].shape == (3,))
        check("load: telem / cmd scalar columns", out["telem"]["yaw"].shape == (3,) and out["cmd"]["vf"].tolist() == [0.2] and out["cmd"]["arm"].tolist() == [1.0])
        check("load: truth vectors (N,3) / (N,4), bools as float", out["truth"]["person"].shape == (1, 3) and out["truth"]["motors"].shape == (1, 4) and out["truth"]["armed"].tolist() == [1.0])
        check("load: event rows kept as dicts, every kind present", out["event"]["rows"][0]["text"] == "arm" and all(k in out for k in schema.KINDS))
        with open(p / "log.jsonl", "a", encoding="utf-8") as f:
            f.write('{"kind": "state", "t_ms": 1')                   # killed mid-write
        check("truncated last line is skipped, not fatal", len(list(session.iter_rows(p))) == 9 and session.load(p)["state"]["t"].shape == (3,))
        a, b2 = session.SessionLog(root=d, tag="dup"), session.SessionLog(root=d, tag="dup")
        check("two sessions in the same second get different directories", a.path != b2.path and b2.path.name.endswith("_2"))
        a.close(); b2.close()
        print(session.summary(p).splitlines()[1])
        nl = session.open_session(None, "x")
        nl.state({}); nl.close()
        check("open_session(None) -> NullLog, falsy, no-ops", isinstance(nl, session.NullLog) and not nl and nl.path is None)
        old = config.POSITIONING_DIR; config.POSITIONING_DIR = d
        try:
            s1 = session.open_session("", "t"); s2 = session.open_session("named", "t")
            check("open_session('') unnamed, open_session('named') suffixed, under config.POSITIONING_DIR",
                  s1 and s2 and s1.path.parent == pathlib.Path(d) and s1.path.name.endswith("_t") and s2.path.name.endswith("_t_named"))
            s1.close(); s2.close()
        finally:
            config.POSITIONING_DIR = old
    return all(results)


# ---------- (3) ----------
def test_replay():
    results, check = checker("replay")
    with tempfile.TemporaryDirectory() as d:
        p = write_session(d, n=20).path
        r = ReplaySource(p, speed=0).start(); out = r.poll()
        ts = [m["t"] for m in out]
        check("speed 0: every state row on the first poll, t strictly increasing, done", len(out) == 20 and ts == sorted(set(ts)) and r.done and r.poll() == [])
        check("telem rows excluded by default", all("balloon" in m for m in out))
        clock = [0.0]
        r = ReplaySource(p, speed=1.0, clock=lambda: clock[0]).start()
        n0 = len(r.poll()); clock[0] = 0.2; n1 = len(r.poll()); clock[0] = 0.2; n2 = len(r.poll()); clock[0] = 10; n3 = len(r.poll())
        check("speed 1 paces on the recorder clock (t_ms): 1 row at 0 s, 4 by 0.2 s, rest by 10 s", (n0, n1, n2, n3) == (1, 3, 0, 16) and r.done)
        r = ReplaySource(p, speed=0, loop=True, kinds=("state", "telem")).start()
        lap1 = r.poll_kinds(); lap2 = r.poll_kinds()
        t_first = [dat["t"] for k, dat in lap1 if k == "state"][0]; t_first2 = [dat["t"] for k, dat in lap2 if k == "state"][0]
        check("loop: one lap per poll with both kinds, lap 2 shifted by span + 1 s so t never goes back",
              len(lap1) == 40 and len(lap2) == 40 and r.lap == 1 and t_first2 == t_first + r.span_ms + ReplaySource.LAP_GAP_MS and not r.done)
        r = ReplaySource(p, speed=50).start(); t0 = time.monotonic(); n = 0
        while not r.done and time.monotonic() - t0 < 5: n += len(r.poll()); time.sleep(0.001)
        check(f"speed 50: 1.25 s of log in {time.monotonic() - t0:.3f} s", n == 20 and time.monotonic() - t0 < 2)
        try:
            ReplaySource(p, kinds=("cmd",)).start(); empty = False
        except ValueError:
            empty = True
        check("no rows of the requested kinds -> ValueError at start()", empty)
    s = make_source("replay:C:\\data\\positioning\\x")
    check("make_source('replay:C:\\\\...') keeps the Windows path, touches no disk", isinstance(s, ReplaySource) and s.session_dir == "C:\\data\\positioning\\x" and s.rows is None)
    check("make_source udp / udp:5017", make_source("udp").port == 5007 and make_source("udp:5017").port == 5017)
    bad = 0
    for spec in ("sim", "nope", "replay"):
        try: make_source(spec)
        except ValueError: bad += 1
    check("make_source: sim without world, unknown, replay without dir -> ValueError", bad == 3)
    return all(results)


# ---------- (4) ----------
def test_fuse():
    results, check = checker("fuse")
    clock = [1000]
    a, b = ListSource(name="vision"), ListSource(name="uwb")
    f = Fuser([a, b, ListSource(name="vision")], priority=["vision", "uwb"], max_age_ms=500, clock=lambda: clock[0])
    check("duplicate source names made unique", f.names == ["vision", "uwb", "vision'"])
    check("nothing received -> no message", f.poll() == [])
    a.push(state(1, [1, 1, 1], src="vision")); b.push(state(2, [2, 2, 2], [5, 5, 1], 3, src="uwb"))
    m = f.poll()[0]
    check("both fresh: balloon from the priority source, person from the one that has it, t from the balloon source",
          m["balloon"] == [1, 1, 1] and m["person"] == [5, 5, 1] and m["person_id"] == 3 and m["t"] == 1
          and m["src"] == "fused" and m["srcs"] == {"balloon": "vision", "person": "uwb"})
    clock[0] = 1600; b.push(state(3, [3, 3, 3], src="uwb"))
    m = f.poll()[0]
    check("priority source stale -> the fresh one wins", m["balloon"] == [3, 3, 3] and m["srcs"]["balloon"] == "uwb" and m["t"] == 3)
    check("person stale (1000 -> 1600) -> reported null", m["person"] is None and m["person_id"] == -1 and "person" not in m["srcs"])
    clock[0] = 5000; a.push(state(9, None, src="vision"))
    m = f.poll()[0]
    check("a null-only message: everything stale -> honest nulls, t = receipt clock", m["balloon"] is None and m["person"] is None and m["t"] == 5000)
    check("no new input -> []", f.poll() == [])
    return all(results)


# ---------- (5) ----------
def test_venue():
    results, check = checker("venue")
    v = venue.load()
    errors, warnings = venue.validate(v, r_balloon=config.R_BALLOON)
    check(f"{venue.DEFAULT_PATH.relative_to(ROOT)} validates clean", errors == [] and warnings == [], f"errors={errors} warnings={warnings}")
    check("config.ARENA == venue arena, nested 2-tuples of float", config.ARENA == v.arena_tuple() and isinstance(config.ARENA[0], tuple)
          and all(isinstance(x, float) for pair in config.ARENA for x in pair))
    check("config.OBSTACLES is a list of (x, y, r) tuples and concatenates", isinstance(config.OBSTACLES, list)
          and all(isinstance(o, tuple) and len(o) == 3 for o in config.OBSTACLES) and ([(0, 0, 0.1)] + config.OBSTACLES)[1:] == config.OBSTACLES)
    check("config.JUDGES_XY is a 2-tuple (or None), == places['judges']", config.JUDGES_XY == v.place("judges")
          and (config.JUDGES_XY is None or (isinstance(config.JUDGES_XY, tuple) and len(config.JUDGES_XY) == 2)))
    check("config.WANDER_BOX None or ((x0,y0),(x1,y1))", config.WANDER_BOX is None or (len(config.WANDER_BOX) == 2 and len(config.WANDER_BOX[0]) == 2))
    check("config.VENUE_FILE / POSITIONING_DIR exported", str(config.VENUE_FILE).endswith("default.json") and config.POSITIONING_DIR)
    check("place('nope') -> None, obstacle by name", v.place("nope") is None and v.obstacle("judges_table").r == 0.45)

    def errs(mut):
        w = venue.load(); mut(w); return venue.validate(w, r_balloon=config.R_BALLOON)

    e, _ = errs(lambda w: setattr(w, "arena", ((1, 1), (0, 0))));                 check("inverted arena -> error", len(e) == 1 and "min" in e[0])
    e, _ = errs(lambda w: w.places.__setitem__("bad", (4.0, 0.0)));                check("place on the table -> overlap error", any("overlaps" in x for x in e))
    e, _ = errs(lambda w: w.places.__setitem__("bad", (10.0, 0.0)));               check("place outside the arena -> error", any("wall" in x for x in e))
    e, w_ = errs(lambda w: w.places.__setitem__("near", (2.7, 0.0)));              check("place 0.85 m from the table edge -> warning only", e == [] and len(w_) == 1)
    e, _ = errs(lambda w: w.obstacles.append(venue.Obstacle(9, 9, 0.3, "far")));   check("obstacle centre outside the arena -> error", any("outside" in x for x in e))
    e, _ = errs(lambda w: w.obstacles.append(venue.Obstacle(0, 0, 0.3, "judges_table")));  check("duplicate obstacle name -> error", any("used 2" in x for x in e))
    e, _ = errs(lambda w: setattr(w, "wander", ((-3, -3), (0, 0))));               check("wander box outside the arena -> error", any("wander" in x for x in e))
    e, _ = errs(lambda w: setattr(w, "wander", ((-1, -1), (1, 1))));               check("wander box inside -> fine", e == [])
    with tempfile.TemporaryDirectory() as d:
        w = venue.load(); w.places["home"] = (0.0, 0.0); w.wander = ((-1.0, -1.0), (1.0, 1.0)); w.obstacles.append(venue.Obstacle(1.0, 1.0, 0.2, "pillar"))
        path = venue.save(w, pathlib.Path(d) / "x.json"); w2 = venue.load(path)
        check("save/load round-trip", w2.arena_tuple() == w.arena_tuple() and w2.obstacle_tuples() == w.obstacle_tuples() and w2.places == w.places
              and w2.wander_tuple() == w.wander and w2.name == w.name and not path.with_suffix(".json.tmp").exists())
        try:
            venue.load(pathlib.Path(d) / "missing.json"); missing = False
        except FileNotFoundError:
            missing = True
        (pathlib.Path(d) / "bad.json").write_text("{\"arena\": [1,", encoding="utf-8")
        try:
            venue.load(pathlib.Path(d) / "bad.json"); badjson = False
        except ValueError as ex:
            badjson = "invalid JSON" in str(ex)
        check("missing file -> FileNotFoundError, broken JSON -> ValueError naming the line", missing and badjson)
    return all(results)


# ---------- (6) ----------
def test_udp():
    results, check = checker("udp")
    src = UdpStateSource(port=5017).start(); tx = UdpJson()
    for i in range(5):
        tx.send(state(i, [i, 0, 1.7]), ("127.0.0.1", 5017))
    time.sleep(0.05)
    got = src.poll()
    check("5 datagrams -> 5 messages in one poll, in order (recv_all)", [m["t"] for m in got] == [0, 1, 2, 3, 4])
    check("nothing pending -> []", src.poll() == [])
    src.stop(); tx.sock.close()
    check("stop closes the socket", src.sock is None)
    return all(results)


# ---------- (7) ----------
def test_evaluate():
    results, check = checker("evaluate")
    with tempfile.TemporaryDirectory() as d:
        p = write_eval_session(d)
        dd = session.load(p)
        rep = evaluate.evaluate(dd); m = rep["metrics"]
        print(evaluate.format_report(rep, "synthetic").splitlines()[-1])
        check(f"latency recovered {m['latency_ms']} ms (truth 120)", m["latency_ms"] is not None and abs(m["latency_ms"] - 120) <= 10)
        check(f"fix age {m['age_ms']:.0f} ms == 120", abs(m["age_ms"] - 120) < 1)
        check(f"rms xy de-lagged {m['rms_xy'] * 100:.1f} cm < 4, raw {m['rms_xy_raw'] * 100:.1f} cm > 5", m["rms_xy"] < 0.04 and m["rms_xy_raw"] > 0.05)
        check(f"rms z {m['rms_z'] * 100:.1f} cm < 4", m["rms_z"] < 0.04)
        check(f"null fraction {m['balloon']['null_frac']:.3f} ~ 0.05", abs(m["balloon"]["null_frac"] - 0.05) < 0.02)
        check(f"balloon jumps {m['balloon_jumps']} == 2 (one outlier row, out and back)", m["balloon_jumps"] == 2)
        check(f"telem max gap {m['telem_rate']['max_gap_ms']:.0f} ms ~ 700-800", 650 <= m["telem_rate"]["max_gap_ms"] <= 800)
        check(f"cmd rate {m['cmd_rate']['hz']:.1f} Hz ~ 15", abs(m["cmd_rate"]["hz"] - 15) < 0.5)
        check("person rms vs truth < 4 cm", m["person_rms_xy"] < 0.04)
        check("outlier -> jumps check FAIL -> not passed", not evaluate.passed(rep))
        p2 = write_eval_session(d, outlier=False, tag="clean")
        rep2 = evaluate.evaluate(session.load(p2))
        check("clean session -> PASS", evaluate.passed(rep2), evaluate.format_report(rep2).splitlines()[-1])
        p3 = write_eval_session(d, outlier=False, noise=0.10, tag="noisy")
        check("10 cm noise -> FAIL on rms", not evaluate.passed(evaluate.evaluate(session.load(p3))))
        p4 = write_eval_session(d, outlier=False, static=True, tag="static")
        rep4 = evaluate.evaluate(session.load(p4))
        lat = [ok for n, v, ok in rep4["checks"] if n.startswith("vision latency")][0]
        check("static truth -> latency check skipped (info), still PASS", rep4["metrics"]["latency_ms"] is None and lat is None and evaluate.passed(rep4))
        e = evaluate.expect_check(session.load(p4)["state"], "balloon", (1.0, 0.5, 1.7))
        e2 = evaluate.expect_check(session.load(p4)["state"], "balloon", (1.3, 0.5, 1.7))
        check(f"--expect: median within {e['dist'] * 100:.1f} cm OK, 30 cm off -> FAIL", e["ok"] and not e2["ok"])
        p5 = write_eval_session(d, outlier=False, tag="follow", t_shift=123456, with_truth=False)
        d5 = session.load(p5); d2 = session.load(p2)
        tr = evaluate.truth_from(d5, d2)
        rep5 = evaluate.evaluate(d5, truth=tr)
        check(f"wall-time join: rms {rep5['metrics']['rms_xy'] * 1000:.2f} mm vs {rep2['metrics']['rms_xy'] * 1000:.2f} mm same-session",
              abs(rep5["metrics"]["rms_xy"] - rep2["metrics"]["rms_xy"]) < 0.001 and abs(rep5["metrics"]["latency_ms"] - rep2["metrics"]["latency_ms"]) <= 5)
        check("no truth and no --expect -> nothing judged, report says so", evaluate.evaluate(d5)["has_truth"] is False and "no truth" in evaluate.format_report(evaluate.evaluate(d5)))
        try:
            evaluate.truth_from(session.load(write_eval_session(d, tag="far", wall0=1.6e9, with_truth=False)), d2); far = False
        except ValueError:
            far = True
        check("sessions that do not overlap in wall time -> ValueError", far)
        if importlib.util.find_spec("matplotlib"):
            png = pathlib.Path(d) / "eval.png"
            evaluate.save_plot(d2, rep2, str(png))
            check(f"plot written ({png.stat().st_size // 1024} kB)", png.stat().st_size > 10_000)
        empty = session.SessionLog(root=d, tag="empty"); empty.telem({"t": 1, "yaw": 0}); ep = empty.path; empty.close()
        try:
            evaluate.evaluate(session.load(ep)); zero = False
        except ValueError:
            zero = True
        check("session without state rows -> ValueError", zero)
    return all(results)


if __name__ == "__main__":
    r = [test_schema(), test_session(), test_replay(), test_fuse(), test_venue(), test_udp(), test_evaluate()]
    print("PASS" if all(r) else "FAIL")
    sys.exit(0 if all(r) else 1)
