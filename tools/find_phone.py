"""Find the DroidCam phone on the hotspot: scans the laptop's /24 for port 4747 and prints the stream URL.

  python tools/find_phone.py                 # subnet from the Mac's en0 address (172.20.10.x on an iPhone hotspot)
  python tools/find_phone.py --grab          # also open the first hit and report frame size + fps for 2 s
  python tools/find_phone.py --subnet 192.168.137 --hosts 1-50

Prints the line to paste into laptop/config.py SOURCES["B"]. DroidCam must be running (foreground) on the phone,
on the SAME hotspot as the laptop. Nothing found: the phone is on another network, DroidCam is not started, or the
network isolates clients (eduroam does; use the hotspot).
"""
import argparse, os, socket, subprocess, sys, time
from concurrent.futures import ThreadPoolExecutor
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

PORT = 4747


def my_ip():
    for iface in ("en0", "en1", "bridge100"):
        try:
            ip = subprocess.run(["ipconfig", "getifaddr", iface], capture_output=True, text=True, timeout=2).stdout.strip()
            if ip:
                return ip, iface
        except Exception:
            pass
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80)); return s.getsockname()[0], "?"
    finally:
        s.close()


def is_open(ip, port=PORT, timeout=0.5):
    try:
        with socket.create_connection((ip, port), timeout=timeout):
            return True
    except OSError:
        return False


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--subnet", default=None, help="first three octets, e.g. 172.20.10 (default: from this Mac's address)")
    ap.add_argument("--hosts", default="1-254", help="last-octet range to scan")
    ap.add_argument("--port", type=int, default=PORT)
    ap.add_argument("--grab", action="store_true", help="open the first hit with OpenCV and report size / fps")
    args = ap.parse_args()
    ip, iface = my_ip()
    subnet = args.subnet or ip.rsplit(".", 1)[0]
    lo, hi = (int(x) for x in args.hosts.split("-"))
    print(f"[find_phone] this Mac is {ip} on {iface}; scanning {subnet}.{lo}-{hi}:{args.port} ...")
    if not args.subnet and not subnet.startswith(("172.20.10", "192.168", "10.")):
        print("  (does not look like a hotspot subnet: are you on the phone's hotspot?)")
    hosts = [f"{subnet}.{n}" for n in range(lo, hi + 1) if f"{subnet}.{n}" != ip]
    with ThreadPoolExecutor(max_workers=64) as ex:
        hits = [h for h, ok in zip(hosts, ex.map(lambda h: is_open(h, args.port), hosts)) if ok]
    if not hits:
        print("nothing answers on that port. DroidCam running and in the foreground? Same hotspot? (the app shows its IP)")
        sys.exit(1)
    for h in hits:
        print(f"  {h}  ->  http://{h}:{args.port}/video")
    url = f"http://{hits[0]}:{args.port}/video"
    print(f'\nSOURCES["B"] = "{url}"     # paste into laptop/config.py, or pass --b {url}')
    if args.grab:
        from laptop.vision.streams import Stream
        s = Stream(url, "B").wait_first(timeout=15)
        t0, last, n = time.monotonic(), None, 0
        while time.monotonic() - t0 < 2.0:
            f, t = s.latest()
            if t != last:
                last, n = t, n + 1
            time.sleep(0.005)
        f, _ = s.latest(); s.stop()
        print(f"[grab] {f.shape[1]}x{f.shape[0]}  ~{n / 2.0:.0f} fps   (intrinsics must be made at this size; 404 on /video -> try /mjpegfeed?1280x720)")


if __name__ == "__main__":
    main()
