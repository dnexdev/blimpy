"""Which firmware letter (C D E F) is which motor, and which way does it push: the wizard for calib/motor_map.json.

The box's four outputs are lettered C, D, E, F by the firmware; the laptop thinks in L (rear left), R (rear right),
S (sideways) and V (vertical). After a rewiring nobody knows which is which, and one wrong guess makes "forward" turn
the balloon or "up" push it sideways. This runs each letter for a few seconds while you watch and feel the air:

  python tools/motor_map.py                 # the wizard: C, D, E, F in turn at 30 %, two questions each -> calib/motor_map.json
  python tools/motor_map.py --check         # afterwards: L, R, S, V "forward" THROUGH the map, so you can confirm each one
  python tools/motor_map.py --send          # push the saved map to the box (the bridge does this on every connection anyway)
  python tools/motor_map.py --pct 40 --secs 4

Gondola on the bench or tied down, props ON, box on, motor battery on, one person with a finger on the power switch.
Nothing else talking to the box (close the bridge first). The wizard writes calib/motor_map.json; laptop/config.py reads
it into config.BLE, the bridge sends it to the box as a MAP line on every connection, and the box keeps it in flash.

Conventions (PROTOCOL.md s5 / s6, seen from behind the gondola looking forward):
  L, R  + = push the balloon FORWARD  -> their air blows BACKWARD (toward the rear)
  S     + = push the balloon LEFT     -> its air blows to the RIGHT
  V     + = push the balloon UP       -> its air blows DOWN
"""
import argparse, json, os, pathlib, sys, time
ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT)); os.chdir(ROOT)
from laptop import config
from laptop.control.ble_gondola import LETTERS, BleakTransport, map_line

ROLES = ("L", "R", "S", "V")
WHERE = {"L": "rear LEFT, on the bar", "R": "rear RIGHT, on the bar", "S": "SIDEWAYS, on the right edge", "V": "VERTICAL, under the middle"}
AIR = {   # role -> (question, answer -> sign)
    "L": ("Where does its air go?   b = BACKWARD, toward the rear (that is +)   f = FORWARD", {"b": 1, "f": -1}),
    "R": ("Where does its air go?   b = BACKWARD, toward the rear (that is +)   f = FORWARD", {"b": 1, "f": -1}),
    "S": ("Where does its air go?   r = to the RIGHT (pushes the balloon LEFT, that is +)   l = to the LEFT", {"r": 1, "l": -1}),
    "V": ("Where does its air go?   d = DOWN (pushes the balloon UP, that is +)   u = UP", {"d": 1, "u": -1}),
}


def ask(prompt, choices):
    while True:
        a = input(prompt + "  > ").strip().lower()
        if a in choices: return a
        print(f"    one of: {' '.join(choices)}")


def spin(tr, line, secs):
    """Send `line` every 200 ms for `secs` seconds (the firmware timeouts must not cut it short), then STOP."""
    t0 = time.monotonic()
    while time.monotonic() - t0 < secs:
        tr.send(line); time.sleep(0.2)
    stop(tr)


def stop(tr):
    t0 = time.monotonic()
    while not tr.send("STOP") and time.monotonic() - t0 < 15: time.sleep(0.2)
    time.sleep(0.3)


def connect():
    tr = BleakTransport().start(); t0 = time.monotonic()
    while not tr.connected and time.monotonic() - t0 < 30: time.sleep(0.1)
    if not tr.connected: raise SystemExit("[map] could not connect to the box (powered? advertising? bridge still running?)")
    return tr


def wizard(tr, pct, secs):
    found = {}                                        # letter -> (role, sign) or None
    print(f"\n[map] Each letter runs at {pct} % for {secs:g} s. Watch which prop turns, then feel where the air goes.\n")
    for letter in LETTERS:
        while True:
            input(f"--- Letter {letter}: press Enter to spin it ({pct} %, {secs:g} s) ")
            spin(tr, f"{letter} {pct}", secs)
            a = ask(f"Which motor turned?   L = {WHERE['L']}   R = {WHERE['R']}   S = {WHERE['S']}   V = {WHERE['V']}   n = none   a = again",
                    ["l", "r", "s", "v", "n", "a"])
            if a == "a": continue
            if a == "n":
                found[letter] = None
                print(f"    {letter}: nothing turned. Try  python tools/motor_map.py --pct {min(100, pct + 20)}  for that one, or check its wires.")
                break
            role = a.upper()
            q, signs = AIR[role]
            s = ask(q + "   a = spin it again", list(signs) + ["a"])
            if s == "a": continue
            found[letter] = (role, signs[s])
            print(f"    {letter} = {role}{'+' if signs[s] > 0 else '-'}")
            break
    return found


def check_found(found):
    roles = [v[0] for v in found.values() if v]
    problems = []
    for r in ROLES:
        n = roles.count(r)
        if n == 0: problems.append(f"no letter was identified as {r} ({WHERE[r]})")
        if n > 1: problems.append(f"{n} letters were called {r}: " + ", ".join(k for k, v in found.items() if v and v[0] == r))
    return problems


def save(found, pct):
    motors = {v[0]: k for k, v in found.items() if v}
    sign = {v[0]: v[1] for k, v in found.items() if v}
    data = {"measured": time.strftime("%Y-%m-%d %H:%M"), "pct": pct,
            "MOTORS": {r: motors[r] for r in ROLES}, "SIGN": {r: sign[r] for r in ROLES},
            "GYRO_SIGN": config.BLE.get("GYRO_SIGN", 1),
            "letters": {k: f"{v[0]}{'+' if v[1] > 0 else '-'}" for k, v in found.items() if v},
            "note": "written by tools/motor_map.py; config.BLE reads it; the bridge sends it to the box as MAP on every connection"}
    path = ROOT / config.MOTOR_MAP_FILE
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")
    config.BLE["MOTORS"], config.BLE["SIGN"] = data["MOTORS"], data["SIGN"]
    print(f"\n[map] written {path.relative_to(ROOT)}")
    print(f"[map] {map_line()}")
    print("[map] for calib/MEASUREMENTS.md:  | " + data["measured"][:10] + " | motor map (tools/motor_map.py) | "
          + " ".join(f"{k}={v}" for k, v in data["letters"].items()) + " | which letter is L/R/S/V and its sign | |")
    return data


def check(tr, pct, secs):
    B = config.BLE
    print(f"\n[map] using {B.get('MAP_SOURCE')}: {map_line()}")
    expect = {"L": "the rear LEFT prop turns, air BACKWARD", "R": "the rear RIGHT prop turns, air BACKWARD",
              "S": "the SIDEWAYS prop turns, air to the RIGHT", "V": "the VERTICAL prop turns, air DOWN"}
    bad = []
    for r in ROLES:
        letter, sign = B["MOTORS"][r], B["SIGN"][r]
        input(f"--- {r} forward = letter {letter} at {sign * pct:+d} %. Expect: {expect[r]}. Enter to run ")
        spin(tr, f"{letter} {sign * pct}", secs)
        a = ask("Was that right?   y / n / a = again", ["y", "n", "a"])
        if a == "a":
            spin(tr, f"{letter} {sign * pct}", secs)
            a = ask("Was that right?   y / n", ["y", "n"])
        if a == "n": bad.append(r)
    input("--- Both rear motors forward (L and R together): both rear props turn, air backward. Enter to run ")
    pcts = {l: 0 for l in LETTERS}
    for r in ("L", "R"): pcts[B["MOTORS"][r]] = B["SIGN"][r] * pct
    spin(tr, "MOTORS " + " ".join(str(pcts[l]) for l in LETTERS), secs)
    if ask("Was that right?   y / n", ["y", "n"]) == "n": bad.append("L+R")
    if bad: print(f"\n[map] WRONG: {', '.join(bad)}. Run the wizard again (python tools/motor_map.py) and redo those.")
    else: print("\n[map] map confirmed. Next: python -m laptop.control.ble_gondola (it sends this map to the box), then teleop.")
    return not bad


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--check", action="store_true", help="run L R S V forward through the saved map and confirm each")
    ap.add_argument("--send", action="store_true", help="send the saved map to the box (MAP line) and exit")
    ap.add_argument("--pct", type=int, default=30, help="percent to run each motor at (default 30; 40-50 if a motor only whirs)")
    ap.add_argument("--secs", type=float, default=3.0)
    args = ap.parse_args()
    pct, secs = max(10, min(100, args.pct)), max(1.0, min(10.0, args.secs))
    print(__doc__)
    tr = connect()
    print(f"[map] connected to {config.BLE['NAME']} at {tr.address}")
    try:
        if args.send:
            line = map_line(); tr.send(line); time.sleep(0.5); print(f"[map] sent: {line}  (the box answers on its serial monitor: 'MAP saved: ...')")
        elif args.check:
            check(tr, pct, secs)
        else:
            while True:
                found = wizard(tr, pct, secs)
                print("\n[map] result: " + "   ".join(f"{k} = {v[0]}{'+' if v[1] > 0 else '-'}" if v else f"{k} = none" for k, v in found.items()))
                problems = check_found(found)
                if not problems: break
                print("[map] not a complete map:\n      " + "\n      ".join(problems))
                if ask("Run the wizard again?   y / n", ["y", "n"]) == "n": return
            save(found, pct)
            tr.send(map_line()); time.sleep(0.3)
            print("[map] sent to the box too. Now confirm it:  python tools/motor_map.py --check")
    except KeyboardInterrupt:
        print("\n[map] interrupted")
    finally:
        stop(tr)
        print("[map] STOP" if tr.connected else "[map] STOP NOT DELIVERED: link is down, cut the motor power")
        tr.close(); tr.wait_closed(5.0)


if __name__ == "__main__":
    main()
