# Video streaming

The rover serves two video paths, picked client-side based on browser support:

- **H.264 over fragmented MP4** via WebSocket + MediaSource Extensions (default).
- **MJPEG** via the legacy `multipart/x-mixed-replace` endpoint (fallback).

The H.264 path uses an `ffmpeg` subprocess running `libx264 -preset ultrafast -tune zerolatency`, broadcast to all WebSocket viewers from a single shared encoder.

## Why H.264 over fMP4

| | MJPEG | H.264 fMP4 |
|---|---|---|
| Per-frame compression only | yes | no (P-frames + GOP) |
| Bitrate at 640×480 15fps | ~6 Mbps | 1–2 Mbps |
| iOS Safari support | yes (`<img>`) | iOS 17.1+ (MSE) |
| Hardware encoder on Pi 5 | n/a | none, software only |
| Encoder CPU | low | similar (libx264 ultrafast) |
| Latency floor | ~80 ms | ~150–300 ms |

The bitrate drop is still the headline. The measured LTE uplink is ~20 Mbps (see `docs/network.md`), which is roomier than the initial ~3 Mbps estimate this work was scoped against — MJPEG at ~6 Mbps would actually fit, but it leaves much less headroom for telemetry, control, and Tailscale overhead, and on metered data plans it burns through the cap fast. H.264 at 1.5 Mbps is ~13× less data per minute.

CPU is roughly a wash on Pi 5 (no hardware H.264 encoder; libx264 is software). The win is bandwidth, not raw CPU. Lower bytes across the wireless stack means less radio time, which on a passively-cooled Pi 5 does reduce thermals — measured by Claude-Rover during QA, not assumed.

WebRTC was considered and rejected for this PR: `aiortc` adds ~100 MB of vendored deps, requires SDP signaling plumbing, and the encoder is still software libx264. Sub-200 ms latency is its main edge; we don't need that for pack-rover teleop. WebRTC remains the upgrade path if we ever do.

## Install

```bash
sudo apt-get update
sudo apt-get install -y ffmpeg
```

Nothing to install in the Python venv. `ffmpeg` is invoked as a subprocess from `app.streamer`.

## Switching modes from the UI

The client picks automatically at page load:

- Browser supports `MediaSource` and `video/mp4; codecs="avc1.42E01F"` → H.264.
- Otherwise → MJPEG.

Force a mode for A/B testing or fallback verification:

- `http://rover.local:8080/?stream=h264` — force H.264 (errors if MSE unavailable).
- `http://rover.local:8080/?stream=mjpeg` — force MJPEG.

If the H.264 WS closes with code `1011` and reason `ffmpeg`, the client falls back to MJPEG and stops retrying — that signals the dep is missing on the rover.

## Runtime config

The encoder accepts width / height / fps / bitrate changes without a service restart.

```bash
# inspect current config
curl -s http://rover.local:8080/api/video/config | jq

# example: dial down for LTE (lower bitrate, smaller frame)
curl -s -X POST http://rover.local:8080/api/video/config \
  -H 'Content-Type: application/json' \
  -d '{"width":480,"height":360,"fps":12,"bitrate_kbps":600}'

# example: max it out for LAN
curl -s -X POST http://rover.local:8080/api/video/config \
  -H 'Content-Type: application/json' \
  -d '{"width":640,"height":480,"fps":15,"bitrate_kbps":2000}'
```

Any change restarts the `ffmpeg` subprocess (~500 ms interruption). All connected clients re-init their MSE buffer from the next init segment and resume on the next keyframe (default GOP is 2 s).

## Rollback path

The MJPEG endpoint at `GET /video.mjpg` is unchanged and remains available:

```html
<img src="/video.mjpg">
```

Any caller that hits it directly still works. Inside the standard UI, force a mode with `?stream=mjpeg` per above. If you want every browser on this rover to default to MJPEG, the cleanest option is a one-line CSS/JS patch in `web/index.html` — the runtime knob is intentionally client-side because the spec said *don't auto-detect uplink type server-side*.

To remove H.264 entirely (revert to pre-PR behavior): drop `app/streamer.py`, remove the `/ws/video` and `/api/video/config` endpoints from `app/main.py`, restore `<img id="video" src="/video.mjpg">` in both `web/*.html`. `git revert <pr-merge-commit>` is the easier path.

## Diagnostics

```bash
# Is the encoder running?
curl -s http://rover.local:8080/api/video/config | jq '.running, .viewers'

# Watch ffmpeg stderr live (logged by the FastAPI server)
journalctl --user -u rover-uvicorn -f | grep ffmpeg

# Manually pull the init segment + a few fragments to verify the path
websocat -b ws://rover.local:8080/ws/video > /tmp/rover.mp4 &
sleep 5; kill %1
ffprobe -hide_banner /tmp/rover.mp4 2>&1 | head

# Measure actual bytes on the wire (Pi side)
sudo iftop -i wlan0 -P -B
```

The shared encoder starts on the first `/ws/video` connection and stops 10 s after the last viewer disconnects. There's no encoder cost while nobody's watching, which is the biggest CPU and thermal win over the always-on MJPEG default.

## Known limitations

- **No hardware H.264 encoder on Pi 5.** The VideoCore VII pipeline dropped HW encode versus the Pi 4. Everything here is software libx264.
- **Bbox overlay assumes 640×480 source.** If you reconfigure the encoder to a different resolution, the dog-bbox overlay in `index.html` will be misaligned (the detector still runs at native camera resolution, but the UI scales the bbox to whatever `<video>` is rendering at). Fix is a one-line ratio adjustment in `reflectBbox` — deferred until we actually change the default.
- **Reconfigure interrupts every viewer.** Each `POST /api/video/config` restarts ffmpeg. The MSE clients reconnect within ~1–2 s. Don't do this on a hot path; it's a knob, not a slider.
- **MSE on iOS Safari requires 17.1+.** Earlier iOS falls back to MJPEG automatically. WebRTC would solve this if we ever care about old phones.
