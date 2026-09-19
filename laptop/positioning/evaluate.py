"""Evaluate a recorded positioning session: errors vs truth, latency, dropouts, jumps, link rates; optional PNG.

  python tools/positioning_eval.py data/positioning/<fake session> --plot             # sim: truth rows in the same session
  python tools/positioning_eval.py <follow session> --truth-session <fake session>    # two sessions recorded together, joined on wall time
  python tools/positioning_eval.py <real session> --expect balloon 1.20 0.40 1.70     # tape measure vs the median fix (static balloon)

  from laptop.positioning import session, evaluate
  d = session.load(dir); rep = evaluate.evaluate(d); print(evaluate.format_report(rep, dir)); evaluate.passed(rep)

Clocks. State rows are aligned to truth on the RECORDER clock `t` (what the controller experienced), never on data.t
(`t_src`): the sim stamps a state message at capture time and the truth row at drain time, so aligning on t_src would
cancel the latency. `age_ms` = median(t - t_src) is the direct latency measurement whenever sender and recorder share
a clock (sim, localize.py and mono.py on the same laptop); the lag scan against truth is the independent one, and the
two must agree within the jitter. Two sessions are joined on `wall` (time.time(), ~16 ms resolution on Windows).
A fake session writes one truth row per drain batch, so truth is always interpolated at the fix times, never zipped.
A follow_me --log session records what the controller CONSUMED (recv_latest at 15 Hz), so judge fix rate and dropouts
on the fake / record session and command gaps on the follow session; the report header names the session tag.
"""
import math
import numpy as np

THRESHOLDS = dict(rms_xy_m=0.06, p95_xy_m=0.15, rms_z_m=0.08, person_p95_xy_m=0.15, latency_ms=(0.0, 250.0),
                  balloon_jumps=0, cmd_max_gap_ms=500.0)


def fixes(state, obj="balloon"):
    """(t, xyz) of the rows where `obj` was seen. Empty arrays when never."""
    p, t = state.get(obj), state.get("t")
    if not isinstance(p, np.ndarray) or p.ndim != 2 or t is None or len(t) == 0:
        return np.zeros(0), np.zeros((0, 3))
    ok = np.isfinite(p[:, 0])
    return np.asarray(t, float)[ok], p[ok]


def resample(t_query, t_ref, p_ref):
    """Reference track interpolated at t_query (NaN outside its span). NaN rows and duplicate times are dropped first."""
    t_ref, p_ref = np.asarray(t_ref, float), np.asarray(p_ref, float)
    if p_ref.ndim == 1:
        p_ref = p_ref[:, None]
    ok = np.isfinite(p_ref).all(axis=1) & np.isfinite(t_ref)
    t_ref, p_ref = t_ref[ok], p_ref[ok]
    order = np.argsort(t_ref, kind="stable"); t_ref, p_ref = t_ref[order], p_ref[order]
    keep = np.concatenate([[True], np.diff(t_ref) > 0]) if len(t_ref) else np.zeros(0, bool)
    t_ref, p_ref = t_ref[keep], p_ref[keep]
    t_query = np.asarray(t_query, float)
    out = np.full((len(t_query), p_ref.shape[1] if p_ref.ndim == 2 else 1), np.nan)
    if len(t_ref) < 2:
        return out
    inside = (t_query >= t_ref[0]) & (t_query <= t_ref[-1])
    for k in range(p_ref.shape[1]):
        out[inside, k] = np.interp(t_query[inside], t_ref, p_ref[:, k])
    return out


def truth_track(truth):
    """(t, balloon xyz (N,3), person xyz (N,3) | None) from a session's truth dict, or None when there is no truth."""
    if not truth or "x" not in truth or len(truth.get("t", ())) < 2:
        return None
    xyz = np.column_stack([truth["x"], truth["y"], truth["z"]])
    person = truth.get("person")
    return np.asarray(truth["t"], float), xyz, person if isinstance(person, np.ndarray) and person.ndim == 2 else None


def truth_from(d_main, d_truth):
    """Truth of ANOTHER session (fake_esp32 --log) mapped onto d_main's recorder clock through wall time.
    Returns a truth dict like d_main["truth"]; ValueError when the sessions do not overlap."""
    st, tr = d_main["state"], d_truth["truth"]
    if len(st["t"]) == 0 or len(tr["t"]) == 0:
        raise ValueError("need state rows in the main session and truth rows in the truth session")
    off = float(np.median(st["wall"] * 1000.0 - st["t"]))            # main session: wall_ms = t + off
    t_mapped = tr["wall"] * 1000.0 - off
    if t_mapped[-1] < st["t"][0] or t_mapped[0] > st["t"][-1]:
        raise ValueError("the two sessions do not overlap in wall time: were they recorded together?")
    out = dict(tr); out["t"] = t_mapped
    return out


def position_errors(t_fix, p_fix, t_truth, p_truth, lag_ms=0.0):
    """Fix minus truth evaluated at t_fix - lag_ms. -> dict(n, rms_xy, p95_xy, max_xy, rms_axis (3,), rms_z, err_t, err_xy)."""
    tr = resample(np.asarray(t_fix, float) - lag_ms, t_truth, p_truth)
    e = np.asarray(p_fix, float) - tr
    ok = np.isfinite(e).all(axis=1)
    e, t = e[ok], np.asarray(t_fix, float)[ok]
    if len(e) == 0:
        return dict(n=0, rms_xy=math.nan, p95_xy=math.nan, max_xy=math.nan, rms_axis=np.full(3, np.nan), rms_z=math.nan, err_t=t, err_xy=np.zeros(0))
    exy = np.hypot(e[:, 0], e[:, 1])
    return dict(n=int(len(e)), rms_xy=float(np.sqrt(np.mean(exy ** 2))), p95_xy=float(np.percentile(exy, 95)), max_xy=float(exy.max()),
                rms_axis=np.sqrt(np.mean(e ** 2, axis=0)), rms_z=float(np.sqrt(np.mean(e[:, 2] ** 2))), err_t=t, err_xy=exy)


def path_length(p):
    p = np.asarray(p, float)
    p = p[np.isfinite(p).all(axis=1)]
    return float(np.sum(np.hypot(np.diff(p[:, 0]), np.diff(p[:, 1])))) if len(p) > 1 else 0.0


def estimate_latency_ms(t_fix, p_fix, t_truth, p_truth, max_lag_ms=500, step_ms=5, min_path_m=1.0):
    """Lag (ms, + = fixes lag truth) that minimises the xy rms against truth, scanned -50..max_lag. (None, None) when
    the truth barely moved (< min_path_m): a static balloon says nothing about latency."""
    if path_length(p_truth) < min_path_m or len(t_fix) < 5:
        return None, None
    best = None
    for lag in range(-50, int(max_lag_ms) + 1, int(step_ms)):
        r = position_errors(t_fix, p_fix, t_truth, p_truth, lag)
        if r["n"] >= 5 and (best is None or r["rms_xy"] < best[1]):
            best = (float(lag), r["rms_xy"])
    return best if best else (None, None)


def dropouts(state, obj="balloon"):
    t = np.asarray(state.get("t", ()), float)
    tf, _ = fixes(state, obj)
    span = (t[-1] - t[0]) / 1000.0 if len(t) > 1 else 0.0
    return dict(n_rows=int(len(t)), n_fix=int(len(tf)), null_frac=float(1 - len(tf) / len(t)) if len(t) else math.nan,
                fix_hz=float(len(tf) / span) if span > 0 else math.nan,
                longest_gap_ms=float(np.diff(tf).max()) if len(tf) > 1 else math.nan)


def jumps(t_fix, p_fix, thresh_m=0.5, max_dt_ms=500):
    """Consecutive fixes further apart than thresh_m within max_dt_ms (a gap is not a jump). An outlier counts twice."""
    if len(t_fix) < 2:
        return 0
    d = np.hypot(np.diff(p_fix[:, 0]), np.diff(p_fix[:, 1])); dt = np.diff(t_fix)
    return int(np.sum((d > thresh_m) & (dt < max_dt_ms)))


def rate_and_gaps(t):
    t = np.asarray(t, float)
    if len(t) < 2:
        return dict(n=int(len(t)), hz=math.nan, median_dt_ms=math.nan, max_gap_ms=math.nan)
    dt = np.diff(t)
    return dict(n=int(len(t)), hz=float(len(t) / ((t[-1] - t[0]) / 1000.0)), median_dt_ms=float(np.median(dt)), max_gap_ms=float(dt.max()))


def age_ms(state):
    """median(t - t_src) over fix rows: the direct latency when sender and recorder share a clock; None on a foreign clock."""
    ts = state.get("t_src")
    if not isinstance(ts, np.ndarray) or len(ts) == 0:
        return None
    a = np.asarray(state["t"], float) - ts
    a = a[np.isfinite(a)]
    if len(a) == 0:
        return None
    m = float(np.median(a))
    return None if abs(m) > 5000 else m


def expect_check(state, obj, xyz, tol_m=0.10):
    """Median fix of a STATIC object vs a tape-measured world point."""
    _, p = fixes(state, obj)
    if len(p) == 0:
        return dict(n=0, median=None, delta=None, dist=math.nan, ok=False)
    med = np.median(p, axis=0); delta = med - np.asarray(xyz, float); dist = float(np.linalg.norm(delta))
    return dict(n=int(len(p)), median=med, delta=delta, dist=dist, ok=dist <= tol_m)


def evaluate(d, truth=None, expect=(), thr=THRESHOLDS, jump_m=0.5, tol_m=0.10):
    """-> {"tag", "has_truth", "metrics", "checks": [(name, value_str, ok)]} with ok True / False / None (= information only)."""
    st = d["state"]
    if len(st.get("t", ())) == 0:
        raise ValueError("no state rows in this session (nothing was received on 5007)")
    truth = d.get("truth") if truth is None else truth
    tt = truth_track(truth)
    m, checks = {}, []
    tb, pb = fixes(st, "balloon"); tp, pp = fixes(st, "person")
    m["balloon"] = dropouts(st, "balloon"); m["person"] = dropouts(st, "person")
    m["balloon_jumps"] = jumps(tb, pb, jump_m); m["person_jumps"] = jumps(tp, pp, jump_m)
    m["age_ms"] = age_ms(st)
    checks.append(("state rows / balloon fixes / person fixes", f"{m['balloon']['n_rows']} / {m['balloon']['n_fix']} / {m['person']['n_fix']}", None))
    checks.append(("balloon fix rate / null fraction / longest gap", f"{m['balloon']['fix_hz']:.1f} Hz / {m['balloon']['null_frac'] * 100:.0f} % / {m['balloon']['longest_gap_ms']:.0f} ms", None))
    checks.append(("balloon jumps (> %.1f m between fixes)" % jump_m, str(m["balloon_jumps"]), m["balloon_jumps"] <= thr["balloon_jumps"] if tt else None))
    checks.append(("person jumps (info: REAL sim injects outliers)", str(m["person_jumps"]), None))
    checks.append(("fix age = t - t_src (direct latency, same-clock senders)", "n/a" if m["age_ms"] is None else f"{m['age_ms']:.0f} ms", None))
    m["has_truth"] = tt is not None
    if tt is not None and len(tb) >= 2:
        t_tr, p_tr, person_tr = tt
        lag, _ = estimate_latency_ms(tb, pb, t_tr, p_tr)
        raw = position_errors(tb, pb, t_tr, p_tr, 0.0)
        best = position_errors(tb, pb, t_tr, p_tr, lag or 0.0)
        m.update(latency_ms=lag, rms_xy_raw=raw["rms_xy"], rms_xy=best["rms_xy"], p95_xy=best["p95_xy"], max_xy=best["max_xy"],
                 rms_axis=best["rms_axis"], rms_z=best["rms_z"], err_t=best["err_t"], err_xy=best["err_xy"])
        lo, hi = thr["latency_ms"]
        checks.append(("vision latency vs truth (lag scan)", "static truth: not observable" if lag is None else f"{lag:.0f} ms",
                       None if lag is None else lo <= lag <= hi))
        checks.append(("balloon rms xy vs truth, de-lagged (raw)", f"{best['rms_xy'] * 100:.1f} cm ({raw['rms_xy'] * 100:.1f} cm)", best["rms_xy"] < thr["rms_xy_m"]))
        checks.append(("balloon p95 / max xy error", f"{best['p95_xy'] * 100:.1f} / {best['max_xy'] * 100:.1f} cm", best["p95_xy"] < thr["p95_xy_m"]))
        checks.append(("balloon rms per axis x / y / z", " / ".join(f"{v * 100:.1f}" for v in best["rms_axis"]) + " cm", best["rms_z"] < thr["rms_z_m"]))
        if person_tr is not None and len(tp) >= 2:
            pe = position_errors(tp, pp, t_tr, person_tr, lag or 0.0)
            m.update(person_rms_xy=pe["rms_xy"], person_p95_xy=pe["p95_xy"])
            checks.append(("person rms / p95 xy vs truth", f"{pe['rms_xy'] * 100:.1f} / {pe['p95_xy'] * 100:.1f} cm", pe["p95_xy"] < thr["person_p95_xy_m"]))
    for kind in ("telem", "cmd"):
        r = rate_and_gaps(d.get(kind, {}).get("t", ()))
        m[f"{kind}_rate"] = r
        ok = None
        if kind == "cmd" and r["n"] >= 2:
            ok = r["max_gap_ms"] < thr["cmd_max_gap_ms"]
        checks.append((f"{kind} rows / rate / max gap", f"{r['n']} / {r['hz']:.1f} Hz / {r['max_gap_ms']:.0f} ms" if r["n"] >= 2 else f"{r['n']}", ok))
    m["expect"] = []
    for obj, x, y, z in expect:
        e = expect_check(st, obj, (x, y, z), tol_m); m["expect"].append((obj, e))
        val = "no fixes" if e["median"] is None else f"median ({e['median'][0]:.2f}, {e['median'][1]:.2f}, {e['median'][2]:.2f}) off by {e['dist'] * 100:.0f} cm"
        checks.append((f"{obj} vs tape measure ({x}, {y}, {z})", val, e["ok"]))
    return {"tag": (d.get("meta") or {}).get("tag"), "name": (d.get("meta") or {}).get("name"), "has_truth": tt is not None, "metrics": m, "checks": checks}


def passed(rep):
    return all(ok is not False for _, _, ok in rep["checks"])


def format_report(rep, title=""):
    lines = [f"[eval] {title}  tag={rep.get('tag')} name={rep.get('name')}  truth={'yes' if rep['has_truth'] else 'no'}"]
    for name, val, ok in rep["checks"]:
        lines.append(f"[eval] {name:52s} {val:>36s}  {'OK' if ok else 'FAIL' if ok is False else '-'}")
    lines.append("PASS" if passed(rep) else "FAIL")
    if not rep["has_truth"]:
        lines[-1] += "  (no truth: nothing judged except --expect)"
    return "\n".join(lines)


def save_plot(d, rep, path, truth=None):
    """2x2 PNG: xy track (+ truth, person, arena), z vs t (+ truth, ToF alt), error or fix age vs t, commands vs t."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    st = d["state"]; truth = d.get("truth") if truth is None else truth
    tt = truth_track(truth)
    t0 = float(st["t"][0]); sec = lambda t: (np.asarray(t, float) - t0) / 1000.0
    tb, pb = fixes(st, "balloon"); tp, pp = fixes(st, "person")
    fig, ax = plt.subplots(2, 2, figsize=(13, 9))
    a = ax[0, 0]
    arena = ((d.get("meta") or {}).get("config") or {}).get("ARENA")
    if arena:
        (x0, y0), (x1, y1) = arena; a.plot([x0, x1, x1, x0, x0], [y0, y0, y1, y1, y0], "k-", lw=1, alpha=0.5, label="arena")
    if tt is not None:
        a.plot(tt[1][:, 0], tt[1][:, 1], "-", color="0.5", lw=1, label="truth")
    if len(pb):
        s = a.scatter(pb[:, 0], pb[:, 1], c=sec(tb), s=6, cmap="viridis", label="balloon fixes"); fig.colorbar(s, ax=a, label="t (s)")
    if len(pp):
        a.plot(pp[:, 0], pp[:, 1], ".", color="tab:red", ms=2, alpha=0.4, label="person fixes")
    a.set_aspect("equal", "datalim"); a.set_xlabel("x (m)"); a.set_ylabel("y (m)"); a.legend(fontsize=8); a.set_title("top view")
    a = ax[0, 1]
    if len(pb): a.plot(sec(tb), pb[:, 2], ".", ms=3, label="balloon z (fix)")
    if tt is not None: a.plot(sec(tt[0]), tt[1][:, 2], "-", color="0.5", lw=1, label="truth z")
    tel = d.get("telem", {})
    alt = tel.get("alt")
    if isinstance(alt, np.ndarray) and np.any(np.isfinite(alt) & (alt > 0)):
        ok = np.isfinite(alt) & (alt > 0); a.plot(sec(tel["t"][ok]), alt[ok], ".", ms=2, alpha=0.5, label="alt (ToF, lens to floor)")
    a.set_xlabel("t (s)"); a.set_ylabel("z (m)"); a.legend(fontsize=8); a.set_title("height")
    a = ax[1, 0]
    if rep["has_truth"] and len(rep["metrics"].get("err_t", ())):
        a.plot(sec(rep["metrics"]["err_t"]), rep["metrics"]["err_xy"] * 100, ".", ms=3)
        a.set_ylabel("xy error vs truth (cm)"); a.set_title(f"error at lag {rep['metrics'].get('latency_ms')} ms")
    else:
        ts = st.get("t_src")
        if isinstance(ts, np.ndarray) and len(ts):
            a.plot(sec(st["t"]), np.asarray(st["t"], float) - ts, ".", ms=3)
        a.set_ylabel("fix age t - t_src (ms)"); a.set_title("fix age")
    a.set_xlabel("t (s)")
    a = ax[1, 1]
    cmd = d.get("cmd", {})
    if len(cmd.get("t", ())):
        for k in ("vf", "vs", "yr", "vz"):
            if k in cmd: a.plot(sec(cmd["t"]), cmd[k], lw=1, label=k)
        if "arm" in cmd: a.plot(sec(cmd["t"]), cmd["arm"] * 0.5, "k--", lw=0.8, label="arm/2")
    for te, row in zip(d.get("event", {}).get("t", ()), d.get("event", {}).get("rows", ())):
        a.axvline(sec([te])[0], color="tab:red", alpha=0.3); a.text(sec([te])[0], 0.45, str(row.get("text", ""))[:12], fontsize=7, rotation=90)
    a.set_xlabel("t (s)"); a.set_ylabel("duty"); a.legend(fontsize=8, ncol=5); a.set_title("commands")
    fig.suptitle(f"{path}"); fig.tight_layout(); fig.savefig(path, dpi=90); plt.close(fig)
    return path
