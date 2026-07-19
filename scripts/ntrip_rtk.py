#!/usr/bin/env python3
"""NTRIP client for the Iowa DOT RTN — feeds RTK corrections to the ZED-F9R.

Replaces the legacy ROS chain (ublox_dgnss -> fix_to_nmea -> ntrip_client)
with one process and no middleware: pull RTCM3 from the caster, write it to
the receiver's USB port, and answer the caster's VRS handshake with honest
GGA sentences built from the receiver's own NAV-PVT/NAV-DOP.

Caster etiquette — the old stack got us flagged as a possible attacker by
the network operator, so this client is deliberately polite:
  * GGA at a fixed slow cadence (default every 10 s), not per-fix. The old
    bridge sent one per /fix message (up to 10 Hz) — a firehose.
  * GGA fields are REAL: actual fix quality (incl. 4/5 when RTK converges),
    actual satellite count, actual HDOP. The old bridge hardcoded
    quality=1/sats=12/HDOP=1.0, which reads as a spoofed client.
  * No GGA until the receiver has a valid fix — never null-island zeros.
  * One connection, exponential backoff on reconnect (5 s doubling to 80 s).
    No hammering the caster when WiFi drops or the fix is lost.
  * Honest User-Agent so the operator can see what/who this is.

Usage:
    ./scripts/ntrip_rtk.py                # creds from repo .env
    ./scripts/ntrip_rtk.py --env /path/.env --serial /dev/ttyACM0

Watch the log: "carrier solution" transitions NONE -> FLOAT -> FIXED as RTK
converges. hAcc should drop from meters to centimeters.

Boot sequence (systemd/rover-ntrip.service runs this at power-on):
  1. Wait for the GPS to enumerate (device node may appear seconds after
     the service starts on a cold boot) — patient loop, no crash.
  2. Wait for a valid local GNSS fix — the caster is never contacted
     before we have a real position to report.
  3. Connect and stream; network-down at boot is handled by the normal
     connect backoff. If the GPS drops off USB entirely, exit nonzero and
     let systemd restart us into the wait-for-device loop.
"""

from __future__ import annotations

import argparse
import base64
import os
import select
import signal
import socket
import struct
import sys
import termios
import time
from pathlib import Path

GGA_INTERVAL = 10.0       # seconds between GGA uploads (caster etiquette)
POLL_INTERVAL = 2.0       # seconds between local NAV-PVT/NAV-DOP polls
BACKOFF_START = 5.0
BACKOFF_MAX = 80.0

# Stable udev path — ttyACMn numbering can shift across boots/replugs.
DEFAULT_SERIAL = (
    "/dev/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00"
)

# Status file for the dashboard: the FastAPI app reads this in /telemetry so
# the web UI can show a GPS/RTK chip. tmpfs, atomic replace, ~2 s cadence.
# Consumers treat a stale mtime (> 10 s) as "GPS offline".
STATUS_PATH = Path(os.environ.get("XDG_RUNTIME_DIR", "/tmp")) / "rover-gps-status.json"

FIX_QUALITY = {  # (fixType, carrSoln) -> NMEA GGA quality
    "none": 0, "3d": 1, "dgnss": 2, "float": 5, "fixed": 4,
}
CARR = {0: "NONE", 1: "FLOAT", 2: "FIXED"}


def log(msg: str) -> None:
    print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)


def write_status(fix: str, sats: int = 0, hacc_m: float | None = None,
                 caster: bool = False) -> None:
    """Atomically publish GPS/RTK state for the dashboard. Best-effort —
    a status hiccup must never disturb the correction stream."""
    import json
    try:
        tmp = STATUS_PATH.with_suffix(".tmp")
        tmp.write_text(json.dumps({
            "fix": fix, "sats": sats,
            "hacc_m": round(hacc_m, 2) if hacc_m is not None else None,
            "caster": caster,
        }))
        tmp.replace(STATUS_PATH)
    except Exception:
        pass


def pvt_fix_label(p: dict | None) -> str:
    if not p or not p["valid"]:
        return "none"
    if p["carr"] == 2:
        return "rtk_fixed"
    if p["carr"] == 1:
        return "rtk_float"
    return {2: "2d", 3: "3d", 4: "3d"}.get(p["fix_type"], "none")


def load_env(path: Path) -> dict:
    env = {}
    for line in path.read_text().splitlines():
        line = line.strip()
        if line and not line.startswith("#") and "=" in line:
            k, v = line.split("=", 1)
            env[k.strip()] = v.strip().strip('"')
    return env


# --- UBX helpers -------------------------------------------------------------

def ubx_frame(cls: int, mid: int, payload: bytes = b"") -> bytes:
    body = bytes([cls, mid]) + struct.pack("<H", len(payload)) + payload
    ck_a = ck_b = 0
    for b in body:
        ck_a = (ck_a + b) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return b"\xb5\x62" + body + bytes([ck_a, ck_b])


class Receiver:
    """Owns the serial port. Polls NAV-PVT/NAV-DOP; RTCM bytes are written
    straight through to the receiver."""

    def __init__(self, dev: str) -> None:
        self.fd = os.open(dev, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        attrs = termios.tcgetattr(self.fd)
        attrs[0] = attrs[1] = attrs[3] = 0   # fully raw: no echo back at the GPS
        termios.tcsetattr(self.fd, termios.TCSANOW, attrs)
        self._buf = b""
        self.pvt: dict | None = None
        self.hdop: float | None = None

    def write_rtcm(self, data: bytes) -> None:
        os.write(self.fd, data)

    def poll_nav(self) -> None:
        os.write(self.fd, ubx_frame(0x01, 0x07))  # NAV-PVT
        os.write(self.fd, ubx_frame(0x01, 0x04))  # NAV-DOP

    def pump(self) -> None:
        """Read pending serial data and parse any UBX frames within."""
        try:
            while True:
                chunk = os.read(self.fd, 4096)
                if not chunk:
                    break
                self._buf += chunk
        except BlockingIOError:
            pass
        buf = self._buf
        i = 0
        while i + 8 <= len(buf):
            if buf[i] != 0xB5 or buf[i + 1] != 0x62:
                i += 1
                continue
            ln = struct.unpack_from("<H", buf, i + 4)[0]
            end = i + 6 + ln + 2
            if end > len(buf):
                break                      # incomplete frame — keep for later
            cls, mid = buf[i + 2], buf[i + 3]
            payload = buf[i + 6:i + 6 + ln]
            if cls == 0x01 and mid == 0x07 and ln == 92:
                self._parse_pvt(payload)
            elif cls == 0x01 and mid == 0x04 and ln == 18:
                self.hdop = struct.unpack_from("<H", payload, 12)[0] * 0.01
            i = end
        self._buf = buf[i:]

    def _parse_pvt(self, p: bytes) -> None:
        year, month, day, hour, mn, sec = struct.unpack_from("<HBBBBB", p, 4)
        fix_type, flags, num_sv = p[20], p[21], p[23]
        lon, lat = struct.unpack_from("<ii", p, 24)
        h_ellips, h_msl = struct.unpack_from("<ii", p, 32)
        hacc = struct.unpack_from("<I", p, 40)[0]
        self.pvt = {
            "utc": (hour, mn, sec),
            "valid": bool(flags & 1) and fix_type in (2, 3, 4),
            "fix_type": fix_type,
            "carr": (flags >> 6) & 0x3,
            "num_sv": num_sv,
            "lat": lat * 1e-7,
            "lon": lon * 1e-7,
            "alt_msl": h_msl / 1000.0,
            "geoid_sep": (h_ellips - h_msl) / 1000.0,
            "hacc_m": hacc / 1000.0,
        }

    def gga(self) -> str | None:
        """Honest GGA from the latest NAV-PVT, or None if there's no valid fix."""
        p = self.pvt
        if not p or not p["valid"]:
            return None
        if p["carr"] == 2:
            quality = FIX_QUALITY["fixed"]
        elif p["carr"] == 1:
            quality = FIX_QUALITY["float"]
        elif p["fix_type"] == 4 or p["fix_type"] == 3:
            quality = FIX_QUALITY["3d"]
        else:
            quality = FIX_QUALITY["none"]
        if quality == 0:
            return None
        h, m, s = p["utc"]
        lat, lon = abs(p["lat"]), abs(p["lon"])
        lat_s = f"{int(lat):02d}{(lat - int(lat)) * 60:010.7f}"
        lon_s = f"{int(lon):03d}{(lon - int(lon)) * 60:010.7f}"
        hdop = self.hdop if self.hdop is not None else 99.9
        body = (f"GPGGA,{h:02d}{m:02d}{s:02d}.00,"
                f"{lat_s},{'N' if p['lat'] >= 0 else 'S'},"
                f"{lon_s},{'E' if p['lon'] >= 0 else 'W'},"
                f"{quality},{min(p['num_sv'], 99):02d},{hdop:.1f},"
                f"{p['alt_msl']:.2f},M,{p['geoid_sep']:.2f},M,,")
        ck = 0
        for c in body:
            ck ^= ord(c)
        return f"${body}*{ck:02X}\r\n"

    def close(self) -> None:
        os.close(self.fd)


# --- NTRIP -------------------------------------------------------------------

def ntrip_connect(env: dict) -> socket.socket:
    host, port = env["NTRIP_HOST"], int(env.get("NTRIP_CASTER_PORT", 2101))
    mount = env["NTRIP_MOUNTPOINT"]
    auth = base64.b64encode(
        f"{env['NTRIP_USER']}:{env['NTRIP_PASS']}".encode()).decode()
    s = socket.create_connection((host, port), timeout=15)
    req = (f"GET /{mount} HTTP/1.1\r\n"
           f"Host: {host}:{port}\r\n"
           f"Ntrip-Version: Ntrip/2.0\r\n"
           f"User-Agent: NTRIP rover1-ntrip/1.0 (andrew.meckley1981@gmail.com)\r\n"
           f"Authorization: Basic {auth}\r\n"
           f"Connection: close\r\n\r\n")
    s.sendall(req.encode())
    hdr = b""
    while b"\r\n\r\n" not in hdr and len(hdr) < 4096:
        chunk = s.recv(1024)
        if not chunk:
            break
        hdr += chunk
    head, _, rest = hdr.partition(b"\r\n\r\n")
    first = head.split(b"\r\n", 1)[0].decode(errors="replace")
    if "200" not in first:
        s.close()
        raise ConnectionError(f"caster refused: {first!r}")
    log(f"caster accepted: {first}")
    return s, rest


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    repo_root = Path(__file__).resolve().parent.parent
    ap.add_argument("--env", type=Path, default=repo_root / ".env")
    ap.add_argument("--serial", default=DEFAULT_SERIAL)
    args = ap.parse_args()

    env = load_env(args.env)
    for key in ("NTRIP_HOST", "NTRIP_MOUNTPOINT", "NTRIP_USER", "NTRIP_PASS"):
        if key not in env:
            log(f"missing {key} in {args.env}")
            return 1

    running = True

    def stop(*_a):
        nonlocal running
        running = False
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    # Boot step 1: wait for the GPS to enumerate. On a cold power-on this
    # service can start seconds before the USB device node exists.
    announced = False
    while running and not os.path.exists(args.serial):
        if not announced:
            log(f"waiting for GPS device {args.serial} …")
            announced = True
        write_status("offline")
        time.sleep(2.0)
    if not running:
        return 0
    rx = Receiver(args.serial)
    log(f"opened {args.serial}")

    backoff = BACKOFF_START
    last_carr = None
    while running:
        # Wait for a local fix before touching the caster — no null GGA, ever.
        rx.poll_nav()
        time.sleep(1.0)
        rx.pump()
        p = rx.pvt
        write_status(pvt_fix_label(p), p["num_sv"] if p else 0,
                     p["hacc_m"] if p else None, caster=False)
        if not rx.gga():
            log("waiting for local GNSS fix before connecting…")
            for _ in range(5):
                if not running:
                    return 0
                time.sleep(1.0)
            continue

        try:
            sock, initial = ntrip_connect(env)
        except Exception as e:
            log(f"connect failed: {e} — retrying in {backoff:.0f}s")
            for _ in range(int(backoff)):
                if not running:
                    return 0
                time.sleep(1.0)
            backoff = min(backoff * 2, BACKOFF_MAX)
            continue
        backoff = BACKOFF_START
        if initial:
            rx.write_rtcm(initial)

        rtcm_bytes = 0
        last_gga = 0.0
        last_poll = 0.0
        last_report = time.monotonic()
        try:
            while running:
                r, _, _ = select.select([sock, rx.fd], [], [], 0.5)
                now = time.monotonic()
                if sock in r:
                    data = sock.recv(4096)
                    if not data:
                        raise ConnectionError("caster closed the stream")
                    rx.write_rtcm(data)
                    rtcm_bytes += len(data)
                if rx.fd in r or now - last_poll >= POLL_INTERVAL:
                    if now - last_poll >= POLL_INTERVAL:
                        rx.poll_nav()
                        last_poll = now
                        p = rx.pvt
                        write_status(pvt_fix_label(p), p["num_sv"] if p else 0,
                                     p["hacc_m"] if p else None, caster=True)
                    rx.pump()
                if now - last_gga >= GGA_INTERVAL:
                    gga = rx.gga()
                    if gga:
                        sock.sendall(gga.encode())
                        last_gga = now
                p = rx.pvt
                if p and p["carr"] != last_carr:
                    log(f"carrier solution: {CARR.get(last_carr, '—')} -> "
                        f"{CARR[p['carr']]} (hAcc {p['hacc_m']:.2f} m)")
                    last_carr = p["carr"]
                if now - last_report >= 30.0:
                    if p:
                        log(f"status: {CARR[p['carr']]} | sats {p['num_sv']} | "
                            f"hAcc {p['hacc_m']:.2f} m | rtcm {rtcm_bytes/1024:.0f} KiB")
                    last_report = now
        except (ConnectionError, OSError) as e:
            # If the GPS itself vanished from USB, restart from scratch —
            # systemd brings us back into the wait-for-device loop with a
            # fresh file descriptor.
            if not os.path.exists(args.serial):
                log(f"GPS device gone ({args.serial}) — exiting for restart")
                return 2
            log(f"stream dropped: {e} — reconnecting in {backoff:.0f}s")
            for _ in range(int(backoff)):
                if not running:
                    break
                time.sleep(1.0)
            backoff = min(backoff * 2, BACKOFF_MAX)
        finally:
            try:
                sock.close()
            except Exception:
                pass

    rx.close()
    log("stopped cleanly")
    return 0


if __name__ == "__main__":
    sys.exit(main())
