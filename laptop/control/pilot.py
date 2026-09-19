"""Blimpy pilot: voice/text intents -> behaviors -> commands to the balloon. Replaces follow_me.py's keyboard loop.

  python -m laptop.control.pilot                       # against: python -m laptop.control.fake_esp32 --sim --plot
  python -m laptop.control.pilot                       # real gondola too: run laptop/control/ble_gondola.py first (it answers on 127.0.0.1)
  python -m laptop.control.pilot --voice local         # whisper + ollama push-to-talk only (no cloud)
  python -m laptop.control.pilot --no-voice            # typed commands only (no whisper/ollama needed)
  python -m laptop.control.pilot --log [NAME]          # + record state/telemetry/intents/commands to data/positioning/ (README 1c)

Voice modes (--voice): omni   Qwen3.5-Omni realtime (OMNI Live track): always listening, sees the desk camera, speaks
                              itself, calls set_intent -> behaviors. Needs OMNI_API_KEY. Local stack stays loaded as the
                              standby: if the cloud session is down, 'v' push-to-talk and pyttsx3 take over automatically.
                       local  whisper -> ollama intent JSON -> pyttsx3 (offline). Default when OMNI_API_KEY is not set.
                       none   typed commands only.

Keys: SPACE arm/disarm     v  push-to-talk (press to start, press again to stop; local stack)
      t  type a command    n  heading nudge     m  mute/unmute the cloud mic     ESC quit
Startup does the heading nudge automatically on first arm (same as follow_me).
"""
import argparse, math, os, sys, threading, time
from .. import config
from .protocol import CMD_PORT, STATE_PORT, TELEM_PORT, UdpJson, make_cmd, now_ms, resolve
from .estimator import StateEstimator
from .link import TelemWatchdog
from .behaviors import Behaviors
from .follow_me import nudge_ok
from .keys import ESC, KeyPoller

G = config.FOLLOW


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--esp", default="127.0.0.1")
    ap.add_argument("--voice", choices=["omni", "local", "none"], default=None,
                    help="omni (default when OMNI_API_KEY is set) | local (whisper+ollama) | none")
    ap.add_argument("--no-voice", action="store_true", help="= --voice none")
    ap.add_argument("--omni-cam", default=None, metavar="SPEC",
                    help=f"camera Blimpy sees through in omni mode (default config.OMNI['CAMERA'] = {config.OMNI['CAMERA']!r}; 'none' = ears only)")
    ap.add_argument("--log", nargs="?", const="", default=None, metavar="NAME", help="record state/telemetry/commands to data/positioning/<ts>_pilot[_NAME]/ (laptop/positioning)")
    args = ap.parse_args()
    args.esp = resolve(args.esp)

    from ..voice import tts
    have_key = bool(os.environ.get("OMNI_API_KEY") or os.environ.get("YIBU_API_KEY"))
    mode = "none" if args.no_voice else (args.voice or ("omni" if have_key else "local"))
    voice_ok, local_ok = mode != "none", False
    omni = cam = watcher = rec = None
    pending = []                      # intents produced by background threads (voice, cloud tool calls)
    reports = []                      # focus-watcher reports from the cloud eyes

    intent = stt = None

    def load_local():
        nonlocal intent, stt, rec, local_ok
        try:
            from ..voice import intent as _intent, stt as _stt
            print("[pilot] loading whisper + warming up LLM...")
            _stt.get_model(); _intent.warmup(); rec = _stt.Recorder()
            intent, stt, local_ok = _intent, _stt, True
            print("\n[pilot] local voice ready (v = push-to-talk)")
        except Exception as e:                       # omni mode must not die because ollama is not running
            if mode == "local": raise
            print(f"\n[pilot] local voice unavailable ({e}); no push-to-talk standby")

    if mode == "local": load_local()
    elif mode == "omni": threading.Thread(target=load_local, daemon=True).start()   # standby loads while the cloud talks

    def omni_intent(it):
        """Cloud tool call (websocket thread) -> main loop -> behaviors; waits briefly for what behaviors said."""
        done, box = threading.Event(), {}
        it = dict(it, _text="(omni)", _done=(done, box)); pending.append(it)
        return box.get("reply") or "done" if done.wait(1.5) else "queued"

    if mode == "omni":
        from ..voice.omni import OmniLive
        from ..voice.omni_watch import FocusWatcher
        spec = args.omni_cam or config.OMNI["CAMERA"]
        if spec and str(spec).lower() != "none":
            from ..vision.streams import Stream
            try: cam = Stream(spec, "eyes").wait_first(5)
            except Exception as e: print(f"[pilot] no eyes ({e}); omni runs ears-only")
        frame_fn = (lambda: cam.latest()[0]) if cam else None
        try:
            omni = OmniLive(on_intent=omni_intent, frame_fn=frame_fn, fps=config.OMNI["FPS"],
                            half_duplex=config.OMNI["HALF_DUPLEX"], purpose="pilot").start()
            print(f"[pilot] OMNI live: {omni.model} ({'eyes + ears' if cam else 'ears only'}). Just talk.")
            if cam: watcher = FocusWatcher(frame_fn, on_report=reports.append, interval=config.OMNI["WATCH_S"])
        except Exception as e:
            print(f"[pilot] OMNI unavailable ({e}); " + ("falling back to local push-to-talk (v)" if local_ok else "typed commands only"))
            omni = None

    def say(text):                                   # cloud voice when the session is up, else the offline one
        if omni is not None and omni.ok and omni.say(text): return
        tts.say(text)

    from ..positioning.session import open_session
    state_in, tel_in, cmd_out, keys = UdpJson(STATE_PORT), UdpJson(TELEM_PORT), UdpJson(), KeyPoller()
    log = open_session(args.log, tag="pilot")    # NullLog unless --log: records what this loop consumed, heard and sent
    est = StateEstimator(alpha=G["POS_ALPHA"], beta=G["VEL_BETA"])
    wd = TelemWatchdog()                          # telemetry silence / board-side failsafe -> disarm (laptop/control/link.py)
    beh = Behaviors(say)
    armed, nudge_end, recording = False, 0.0, False
    t_balloon = 0

    def run_text(text):
        if not text.strip(): return
        if local_ok:
            it = intent.parse_intent(text)
        else:                          # minimal offline fallback: intent name typed directly, e.g. "follow_me" or "rotate 90"
            parts = text.split(); it = {"intent": parts[0], "reply": f"{parts[0]} okay."}
            if len(parts) > 1:
                try: it["degrees"] = it["minutes"] = float(parts[1])
                except ValueError: it["target"] = it["direction"] = it["mood"] = parts[1]
        it["_text"] = text
        print(f"\n[pilot] heard {text!r} -> {it}")
        pending.append(it)

    def voice_toggle():
        nonlocal recording
        if not recording:
            rec.start(); recording = True; print("\n[pilot] listening... press v again to stop")
        else:
            audio = rec.stop(); recording = False
            threading.Thread(target=lambda: run_text(stt.transcribe(audio)), daemon=True).start()

    print(__doc__)
    say("Blimpy online.")
    try:
        while True:
            t, now = time.monotonic(), now_ms()
            r = state_in.recv_latest()
            if r:
                s = r[0]; log.state(s)
                if s.get("balloon"): est.update_balloon(s["balloon"], s.get("t", now)); t_balloon = now
                if s.get("person"): beh.on_person(s["person"], s.get("t"))
            for m, _ in tel_in.recv_all(only_from=args.esp):          # every frame: the one that says "failsafe" must not be skipped
                est.update_telem(m.get("yaw", 0.0), m.get("yr", 0.0), m.get("alt"), m.get("t"), now=t); log.telem(m)
                if (warn := wd.telem(m, t, armed)): print(f"\n[pilot] {warn}")

            while (k := keys.poll()) is not None:
                if k == " ":
                    armed = not armed; log.event("arm", armed=armed)
                    if armed:
                        wd.arm(t); beh.on_armed(est)
                        if not est.head_ok and not nudge_ok(est, beh.obstacles, beh.arena): say("I need more room to learn which way I'm facing.")
                elif k == "n" and armed: est.forget_heading(); beh.acq_i, beh.acq_t0, beh.acq_n = 0, None, 0
                elif k == "v" and local_ok: voice_toggle()
                elif k == "m" and omni is not None: omni.muted = not omni.muted; print(f"\n[pilot] cloud mic {'MUTED' if omni.muted else 'open'}")
                elif k == "t":
                    keys.close(); run_text(input("\ncommand> ")); keys.__init__()
                elif k == ESC: raise KeyboardInterrupt

            while reports: beh.on_focus_report(reports.pop(0))
            if watcher is not None: watcher.enabled = beh.focus      # only look at the desk while the focus guard is on
            while pending:
                it = pending.pop(0)
                log.event("intent", said=it.get("_text"), **{k: v for k, v in it.items() if not k.startswith("_") and k != "text"})
                reply = beh.handle(it, est)
                if it.get("_done"):                                   # cloud tool call: the model speaks the result itself
                    done, box = it["_done"]; box["reply"] = reply or "done"; done.set()
                    print(f"\n[pilot] omni set_intent {({k: v for k, v in it.items() if not k.startswith('_')})} -> mode {beh.mode}, {box['reply']!r}")
                elif reply: say(reply)

            vf = vs = yr = vz = 0.0; note = "DISARMED"
            if armed:
                reason = wd.check(armed, t)
                if reason:
                    armed, note = False, f"{reason} -> disarm"; log.event("disarm", reason=reason); say(f"Motors off. {reason}.")
                elif est.p is None or now - t_balloon > G["BALLOON_LOST_MS"]:
                    armed, note = False, "BALLOON LOST -> disarm"; log.event("disarm", reason="balloon lost")
                else:
                    vf, vs, yr, vz, note = beh.step(est)
            est.observe_motion(vf, vs, vz)
            cmd = make_cmd(vf, yr, vz, armed, vs)
            cmd_out.send(cmd, (args.esp, CMD_PORT)); log.cmd(cmd)

            pos = "(%.2f,%.2f,%.2f)" % tuple(est.p) if est.p else "none"
            psi = f"{math.degrees(est.psi):+4.0f}" if est.psi is not None else "n/a"
            cloud = "" if omni is None else (" OMNI" + ("m" if omni.muted else "") if omni.ok else " omni-DOWN")
            print(f"[pilot] {'ARM ' if armed else 'safe'} {note:28s} vf={vf:+.2f} vs={vs:+.2f} yr={yr:+.2f} vz={vz:+.2f} | {pos} psi={psi} z:{est.z_src} "
                  f"{'REC' if recording else '   '}{cloud}      ", end="\r", flush=True)
            time.sleep(max(0.0, 1 / G["HZ"] - (time.monotonic() - t)))
    except KeyboardInterrupt:
        pass
    finally:
        for _ in range(3): cmd_out.send(make_cmd(0, 0, 0, False), (args.esp, CMD_PORT)); time.sleep(0.02)
        if watcher is not None: watcher.stop()
        if omni is not None: omni.stop()
        if cam is not None: cam.stop()
        keys.close(); log.close(); print("\n[pilot] disarmed, bye" + (f"   recorded {log.n} rows -> {log.path}" if log else ""))


if __name__ == "__main__":
    main()
