"""Blimpy pilot: voice/text intents -> behaviors -> commands to the balloon. Replaces follow_me.py's keyboard loop.

  python -m laptop.control.pilot                       # against: python -m archive.fake_esp32 --sim --plot
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
      p  press-to-talk for the cloud: the mic goes up in full for ONE command, past the gate and past mute, until you
         stop talking (or 10 s). Stage mode in a loud hall: press m once, then p before each command.
      l  room: auto -> quiet -> loud. Quiet (judging room): a command said to Blimpy's face needs no name. Loud (the
         floor): the name, or a quick follow-up (the conversation fades faster there). auto decides from the mic's noise floor.
--mic picks the input device (any earbuds with a mic beat the laptop's: list them with  python -m sounddevice), --spk the
output (AirPods as the mic, laptop speakers for the voice:  --mic AirPods --spk Speakers). Keep HALF_DUPLEX on: the
earbud mic still hears the laptop speakers.
Startup does the heading nudge automatically on first arm (same as follow_me).
"""
import argparse, math, os, sys, threading, time
from .. import config
from .protocol import CMD_PORT, STATE_PORT, TELEM_PORT, UdpJson, make_cmd, now_ms, resolve
from .estimator import StateEstimator
from .link import TelemWatchdog
from .altitude import LiftHold
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
    ap.add_argument("--mic", default=None, metavar="DEVICE", help="input device for the cloud mic: index or part of the name from  python -m sounddevice  (earbuds, headset)")
    ap.add_argument("--spk", default=None, metavar="DEVICE", help="output device for Blimpy's voice (e.g. 'Speakers' when AirPods are the mic and Windows moved the output to them)")
    ap.add_argument("--omni-cam", default=None, metavar="SPEC",
                    help=f"camera Blimpy sees through in omni mode (default config.OMNI['CAMERA'] = {config.OMNI['CAMERA']!r}; 'none' = ears only)")
    ap.add_argument("--auto-arm", type=float, default=None, metavar="S", help="arm by itself once the box answers and the balloon has been in view for S seconds "
                    "(a camera window takes the keyboard focus, and SPACE never reaches this terminal). Once per run; SPACE / ESC still work")
    ap.add_argument("--log", nargs="?", const="", default=None, metavar="NAME", help="record state/telemetry/commands to data/positioning/<ts>_pilot[_NAME]/ (laptop/positioning)")
    ap.add_argument("--fpv", default=config.FPV["SOURCE"], metavar="SPEC", help="the eye on the balloon: ESP32-CAM stream URL or camera index (default config.FPV SOURCE); it is also what Blimpy sees in omni mode unless --omni-cam says otherwise")
    ap.add_argument("--no-fpv", action="store_true")
    ap.add_argument("--relative", action="store_true", default=config.FPV["RELATIVE"], help="no room camera: fly on the eye + ultrasonic (follow / rotate / hover-still; go_to / wander refused)")
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
    eye = None
    if args.fpv and not args.no_fpv:
        from ..vision.fpv import FpvEye
        try: eye = FpvEye(args.fpv); print(f"[pilot] eye on the balloon: {args.fpv} (YOLO {config.FPV['WEIGHTS'] or config.YOLO_PERSON})")
        except Exception as e: print(f"[pilot] no eye ({e}); following needs the room camera")

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

    from .scene import Scene
    t_switch = 0                         # now_ms() of the last change of the person Blimpy acts on
    scene, room = Scene(), None          # who is where (room camera's roster + names bound by voice); room = RoomEyes when --omni-cam room

    def psi_ok(): return est.psi is not None and bool(getattr(est, "head_confident", False)) and not est.rel

    def balloon_now():
        """The balloon's position if the room camera has seen it recently, else None: a marker or a distance built on a
        position from a minute ago would be presented to the model as the present."""
        return est.p if est.p is not None and now_ms() - t_balloon <= G["BALLOON_LOST_MS"] else None

    def people_intent(it):
        """Commands that name a PERSON (follow P2, go behind Raymond, "this is Peter"), resolved against the room camera's
        roster BEFORE behaviors sees them. -> a sentence for the model to say, or None = not mine, behaviors handles it.
        Changes `it` in place so that behaviors acts on the chosen person (it only ever knows one). Nothing is switched
        (target, mono's single-person message, the person track) until every check has passed: a refused command
        leaves Blimpy acting on whoever it was acting on."""
        nonlocal t_switch
        k = it.get("intent")
        if k == "name_person":
            if room is None: return "I can't tell people apart without my room camera."
            label, why = scene.resolve(it.get("person"))
            if label is None: return why
            if not scene.bind(label, it.get("name", "")): return "I couldn't catch that name. Say it again?"
            log.event("name_person", label=label, name=scene.names.get(label))
            return f"Got it: {label} is {scene.names[label]}."
        if room is None: return None
        if k == "follow_me": who = it.get("person") or "me"
        elif k == "go_to" and it.get("target", "me") in ("me", "person"): who = it.get("person") or "me"
        else: return None                                  # home, judges, nudge, rotate ...: a stray `person` field changes nothing
        label, why = scene.resolve(who)
        if label is None: return why
        p = scene.get(label); xy = None
        if k == "go_to" and it.get("where") == "behind":
            xy = scene.behind_point(label, balloon_now(), arena=beh.arena, margin=config.R_BALLOON + 0.3)
            if xy is None:
                return (f"I can see {scene.title(label)}, but I can't get behind them: either I don't know exactly where they stand "
                        "(their feet must be in my room camera's picture) or there is no room behind them.")
        elif p is None or p.get("q") != "feet":
            return (f"I can see {scene.title(label)}, but their feet are out of my room camera's picture, so I don't know where they stand. "
                    "Ask them to step back a little.")
        if label != scene.target:
            scene.target = label; room.target = label; beh.reset_person(); t_switch = now_ms()
        log.event("person_target", label=label, who=who)
        if xy is not None: it["target"], it["_xy"] = "point", xy
        elif k == "go_to": it["target"] = "me"
        it["_who"] = scene.title(label)
        return None

    def omni_intent(it):
        """Cloud tool call (websocket thread) -> main loop -> behaviors; waits briefly for what behaviors said."""
        done, box = threading.Event(), {}
        it = dict(it, _text="(omni)", _done=(done, box)); pending.append(it)
        return box.get("reply") or "done" if done.wait(1.5) else "queued"

    if mode == "omni":
        from ..voice.omni import OmniLive
        from ..voice import omni as omni_mod
        from ..voice.omni_watch import FocusWatcher
        spec = args.omni_cam or ("eye" if eye is not None else config.OMNI["CAMERA"])
        frame_fn = raw_fn = None
        if spec == "eye": frame_fn = eye.frame                  # Blimpy sees through its own eye (one stream client only)
        elif str(spec).lower() == "room":
            # The room camera's process (mono.py) owns the webcam and serves its picture + everybody in it; the marks
            # (labels, names, BLIMPY, the strip that says where people are RELATIVE TO BLIMPY) are drawn HERE, where the
            # names and the balloon's pose live. Also the only way to have eyes on Windows while mono runs (one process
            # per webcam).
            from ..vision.eyes import RoomEyes
            room = RoomEyes(hz=config.PEOPLE["ROOM_POLL_HZ"]); room.start()
            print("[pilot] eyes = the room camera (mono.py). Waiting for its picture ...")
            if not room.wait_first(30):
                print("[pilot] WARNING: no picture from the room camera (is  python -m laptop.vision.mono --auto-calib  running?). "
                      "Blimpy is BLIND until it answers; --omni-cam 0 / 1 / none are the other choices")
            marked = {"t": None, "img": None}
            def raw_fn(): return room.latest()[0]
            def frame_fn():
                img, meta = room.latest()
                if img is None: return None
                if meta.get("t") != marked["t"]:              # annotate once per picture, however often it is asked for
                    marked["t"], marked["img"] = meta.get("t"), scene.annotate(img, balloon_now(), est.psi, psi_ok(), meta=meta)
                return marked["img"]
        elif spec and str(spec).lower() != "none":
            from ..vision.streams import Stream
            try: cam = Stream(spec, "eyes").wait_first(5)
            except Exception as e: print(f"[pilot] no eyes ({e}); omni runs ears-only")
            frame_fn = (lambda: cam.latest()[0]) if cam else None
        if raw_fn is None: raw_fn = frame_fn
        O = config.OMNI
        gate = dict(open_db=O["GATE_DB"], min_dbfs=O["GATE_MIN_DBFS"], preroll_ms=O["GATE_PREROLL_MS"], hangover_ms=O["GATE_HANGOVER_MS"],
                    warmup_s=O["GATE_LISTEN_S"], on_ready=lambda g: print(f"\n[pilot] {g.verdict()}")) if O["GATE"] else False
        try:
            name_gate = dict(words=tuple(w.lower() for w in O["NAME_WORDS"]), judge=O["JUDGE"], params=O["ADDRESSEE"],
                             policy=O["ADDRESS"], mode=O["ADDRESS_MODE"], loud_floor_db=O["LOUD_FLOOR_DB"], gate_db_loud=O["GATE_DB_LOUD"],
                             verdict_timeout_s=O["VERDICT_TIMEOUT_S"]) if O["NAME_GATE"] else False

            def facing():
                """Someone near and centred in Blimpy's eye (the FPV observation the behaviors already hold): in a quiet
                room a command said to its face needs no name. No eye, or nobody in it = False."""
                f = beh.fpv if beh.fpv_ok() else None
                if f is None:                                        # no eye on the balloon: the room camera's people near the balloon, or UNKNOWN
                    return scene.near(balloon_now(), O["PRESENCE_RANGE_M"]) if room is not None else None   # (never a made-up "nobody": the judge is told what we say)
                if "facing" in f: return f["facing"]                 # how many people are near and centred (several = nobody in particular)
                return int(abs(f["bearing"]) < O["PRESENCE_BEARING_RAD"] and (f.get("range") is None or f["range"] < O["PRESENCE_RANGE_M"]))
            omni = OmniLive(on_intent=omni_intent, frame_fn=frame_fn, fps=O["FPS"], half_duplex=O["HALF_DUPLEX"], purpose="pilot", gate=gate,
                            mic_device=args.mic, spk_device=args.spk, name_gate=name_gate, presence_fn=facing, record=O["RECORD"],
                            scene_fn=(lambda: scene.snapshot(balloon_now(), est.psi, psi_ok())) if room is not None else None,
                            session={"instructions": omni_mod.instructions(room=room is not None)}).start()   # devices: index or name fragment
            print(f"[pilot] addressing {'%s (%s room): cues + room thresholds + judge %s (voice/addressee.py); a stop word or a p turn always counts; team chatter makes no sound' % (O['ADDRESS'], O['ADDRESS_MODE'], O['JUDGE']) if name_gate else 'off: everything is answered'}")
            if args.mic is not None or args.spk is not None:
                import sounddevice as sd
                print(f"[pilot] cloud mic: {sd.query_devices(omni.mic_device, 'input')['name'][:50]}   voice out: {sd.query_devices(omni.spk_device, 'output')['name'][:50]}")
            print(f"[pilot] OMNI live: {omni.model} ({'eyes (' + spec + ') + ears' if frame_fn else 'ears only'}); "
                  f"mic gate {'on: listening to the room for %.0f s first, then it streams only while someone talks' % O['GATE_LISTEN_S'] if gate else 'OFF: 0.77 units per minute'}.")
            if raw_fn: watcher = FocusWatcher(raw_fn, on_report=reports.append, interval=O["WATCH_S"])       # the desk watcher gets the picture WITHOUT the marks
            try:
                from ..voice.usage_log import relay_balance
                b = relay_balance(omni.key); print(f"[pilot] key ...{omni.key[-4:]}: {b['spent']:.2f} of {b['limit']} units used ({b['pct']:.1f} %)")
            except Exception as e: print(f"[pilot] balance unavailable ({e})")
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
    lift = LiftHold()                                # the height loop (laptop/control/altitude.py), gains config.HOVER
    beh = Behaviors(say)
    armed, nudge_end, recording = False, 0.0, False
    t_balloon = 0; t_eye = None
    t_seen, auto_left = None, args.auto_arm

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
            for s, _ in state_in.recv_all():                  # every row: eye rows are interleaved with the vision rows
                log.state(s)
                if s.get("balloon"):
                    est.update_balloon(s["balloon"], s.get("t", now)); t_balloon = now
                    lift.update(s["balloon"], s.get("t", now) / 1000.0)     # the height loop keeps its own track of the raw fixes
                if s.get("person") and now - t_switch > 600: beh.on_person(s["person"], s.get("t"))   # rows of the PREVIOUS person are still in flight right after a switch
                if s.get("fpv"): beh.on_fpv(s["fpv"], s.get("t"))          # the eye from the sim or a standalone fpv.py
            if eye is not None:
                obs, t_obs = eye.latest()
                if obs is not None and t_obs != t_eye: beh.on_fpv(obs, t_obs); t_eye = t_obs
            for m, _ in tel_in.recv_all(only_from=args.esp):          # every frame: the one that says "failsafe" must not be skipped
                est.update_telem(m.get("yaw", 0.0), m.get("yr", 0.0), m.get("alt"), m.get("t"), now=t); log.telem(m)
                if (warn := wd.telem(m, t, armed)): print(f"\n[pilot] {warn}")
            if args.relative and est.p is None and est.seed_relative(): print("\n[pilot] relative mode: no room camera, flying on the eye + altimeter")

            pressed = []
            while (k := keys.poll()) is not None: pressed.append(k)
            if auto_left is not None and not armed:
                ok = est.p is not None and now - t_balloon < 300 and wd.t_last is not None and t - wd.t_last < 1.0
                t_seen = (t if t_seen is None else t_seen) if ok else None
                if t_seen is not None and t - t_seen >= auto_left: pressed.append(" "); auto_left = None; print("\n[pilot] auto-arm")
            for k in pressed:
                if k == " ":
                    armed = not armed; log.event("arm", armed=armed)
                    if armed:
                        wd.arm(t); beh.on_armed(est); lift.arm()            # hold the height it has right now
                        if not est.head_ok and not est.rel and not nudge_ok(est, beh.obstacles, beh.arena): print("\n[pilot] not much room here to learn the heading (follow / go-to need it; hover and move left / right do not)")
                elif k == "n" and armed: est.forget_heading(); beh.acq_i, beh.acq_t0, beh.acq_n = 0, None, 0
                elif k == "v" and local_ok: voice_toggle()
                elif k == "m" and omni is not None: omni.muted = not omni.muted; print(f"\n[pilot] cloud mic {'MUTED' if omni.muted else 'open'}")
                elif k == "p" and omni is not None: omni.push_to_talk(); print("\n[pilot] press-to-talk: say your command")
                elif k == "l" and omni is not None: print(f"\n[pilot] room: {omni.cycle_mode()} (now {'LOUD' if omni.loud else 'QUIET'})")
                elif k == "t":
                    keys.close(); run_text(input("\ncommand> ")); keys.__init__()
                elif k == ESC: raise KeyboardInterrupt

            if room is not None:
                scene.update(room.latest()[1])
                room.target = scene.target
            while reports: beh.on_focus_report(reports.pop(0))
            if watcher is not None: watcher.enabled = beh.focus      # only look at the desk while the focus guard is on
            while pending:
                it = pending.pop(0)
                log.event("intent", said=it.get("_text"), **{k: v for k, v in it.items() if not k.startswith("_") and k != "text"})
                reply = people_intent(it)
                if reply is None:
                    reply = beh.handle(it, est)
                    if it.get("_who") and it.get("_done") and len(scene.people) > 1:   # several people: the MODEL is told whom Blimpy took, so a wrong
                        reply = f"{reply or 'done'} (acting on {it['_who']}; say so)"    # guess gets corrected out loud (cloud calls only: a typed reply is spoken verbatim)
                if it.get("_done"):                                   # cloud tool call: the model speaks the result itself
                    done, box = it["_done"]; box["reply"] = reply or "done"; done.set()
                    print(f"\n[pilot] omni set_intent {({k: v for k, v in it.items() if not k.startswith('_')})} -> mode {beh.mode}, {box['reply']!r}")
                elif reply: say(reply)

            vf = vs = yr = vz = 0.0; note = "DISARMED"
            if armed:
                reason = wd.check(armed, t)
                if reason:
                    armed, note = False, f"{reason} -> disarm"; log.event("disarm", reason=reason); say(f"Motors off. {reason}.")
                elif est.rel and not est.alt_ok:
                    armed, note = False, "ALTIMETER LOST -> disarm"; log.event("disarm", reason="altimeter lost")
                elif not est.rel and (est.p is None or now - t_balloon > G["BALLOON_LOST_MS"]):
                    armed, note = False, "BALLOON LOST -> disarm"; log.event("disarm", reason="balloon lost")
                else:
                    vf, vs, yr, vz, note = beh.step(est)
                    if not est.rel:                                  # room camera: the height is NOT the behaviours' (or the voice agent's)
                        vz = lift.cmd(t, beh.z_offset)               # business. altitude.LiftHold decides the fan every tick, whatever
                        note = f"{note} | {lift.note}"               # the mode; "go up / down" only moves its target (z_offset)
            est.observe_motion(vf, vs, vz)
            cmd = make_cmd(vf, yr, vz, armed, vs)
            cmd_out.send(cmd, (args.esp, CMD_PORT)); log.cmd(cmd)

            pos = ("rel z=%.2f" % est.p[2] if est.rel else "(%.2f,%.2f,%.2f)" % tuple(est.p)) if est.p else "none"
            if beh.fpv_ok(): pos += " eye %+.0f%s" % (math.degrees(beh.fpv["bearing"]), "" if beh.fpv_r is None else " %.1fm" % beh.fpv_r)
            psi = f"{math.degrees(est.psi):+4.0f}" if est.psi is not None else "n/a"
            cloud = "" if omni is None else ((" OMNI" + ("m " if omni.muted else " ") + omni.mic_status()) if omni.ok else " omni-DOWN")
            print(f"[pilot] {'ARM ' if armed else 'safe'} {note:28s} vf={vf:+.2f} vs={vs:+.2f} yr={yr:+.2f} vz={vz:+.2f} | {pos} psi={psi} z:{est.z_src} "
                  f"{'REC' if recording else '   '}{cloud}      ", end="\r", flush=True)
            time.sleep(max(0.0, 1 / G["HZ"] - (time.monotonic() - t)))
    except KeyboardInterrupt:
        pass
    finally:
        for _ in range(3): cmd_out.send(make_cmd(0, 0, 0, False), (args.esp, CMD_PORT)); time.sleep(0.02)
        if watcher is not None: watcher.stop()
        if omni is not None:
            omni.stop()
            print(f"\n[pilot] cloud this session: {omni.cost['audio_in_s']:.0f} s of audio sent"
                  + (f" of {omni.gate.total_s:.0f} s heard" if omni.gate else "") + f", {omni.cost['audio_out_s']:.0f} s of Blimpy talking, about {omni.units():.2f} units")
        if cam is not None: cam.stop()
        if eye is not None: eye.stop()
        keys.close(); log.close(); print("\n[pilot] disarmed, bye" + (f"   recorded {log.n} rows -> {log.path}" if log else ""))


if __name__ == "__main__":
    main()
