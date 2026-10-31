# Claude Rover: recon for the GPS waypoint plan (2026-10-31)

You are Claude Rover on rover1 (the Pi). Claude Mac has written a plan to make
this rover a true autonomous RTK waypoint follower with back-and-forth looping
(`auto_gps_waypoint_plan.txt`, pull `origin/simplify` if it is there; if not,
the questions below stand on their own). Claude Mac cannot SSH in yet, so you
are the eyes on the hardware. This is a READ-ONLY recon: gather facts, write
them to a handoff note, commit and push on `simplify`.

## Hard rules

- Do NOT write to the motor board (I2C 0x34). No motor commands, not even on
  blocks. Reading its registers is fine.
- Do NOT restart, stop or reconfigure `rover-ntrip` or `rover-uvicorn`
  (user units: `systemctl --user status rover-ntrip rover-uvicorn`).
- Do NOT open the GPS serial port; rover-ntrip owns it. Use the status file
  and the journal instead.
- Writing IMU control registers (0x6a / 0x1c) to enable sampling is allowed.
  Keep the rover stationary while sampling.
- Do not change code. New files only under `docs/rover-state-2026-10-31/`
  plus the handoff note. If something is unavailable, say so and move on.
  Budget: about 45 minutes.

## 1. Identity and state

- `hostname; uptime; git -C ~/ros2_ws/src/rover1 rev-parse --abbrev-ref HEAD; git log --oneline -3; git status --short; git stash list; git branch -vv`
- Python used by the services: `~/code/rover1-venv/bin/python -V`, `pip list` (smbus2, numpy, fastapi versions).
- `systemctl --user status rover-ntrip rover-uvicorn --no-pager | head -40`
- `vcgencmd measure_temp; vcgencmd get_throttled; top -bn1 | head -15`
- Any untracked or local-only files in the repo (the July note mentioned
  `Camera_Specs/COLCON_IGNORE` and `tests/probe_i2c_block_write.py`).

## 2. IMU (the single most important answer)

- `i2cdetect -y 1` (expect 0x34 motor board; BerryIMU v3 would show 0x6a
  LSM6DSL, 0x1c LIS3MDL, 0x77 BMP388).
- WHO_AM_I: read register 0x0F at 0x6a (expect 0x6A) and at 0x1c (expect 0x3D).
- If the LSM6DSL answers: with smbus2, write CTRL2_G (0x11) = 0x4C (104 Hz,
  2000 dps, 70 mdps/LSB) and CTRL1_XL (0x10) = 0x4A, CTRL3_C (0x12) = 0x04,
  then sample gyro (0x22, 6 bytes) and accel (0x28, 6 bytes) at 50 Hz for
  20 s with the rover still. Report per axis: gyro mean and std in deg/s,
  accel mean in g. State which accel axis reads about +-1 g (that is "up").
- Where is the IMU physically mounted right now (roof, chassis, on the Pi
  header)? Cable length, distance to the nearest motor, if you can tell from
  the OS at all (otherwise say "unknown, needs eyes").
- Look for a magnetometer calibration from the Nov 2025 era:
  `find / -xdev -name 'imu_cal.json' 2>/dev/null`.

## 3. Motor board (read-only)

- Read register 0x14 (motor type; 3 = JGB37-520) and 0x15 (encoder polarity)
  as single bytes, and 0x00 (2 bytes, battery mV).
- Read 0x3C as 16 bytes, unpack `<iiii`, twice 5 s apart (rover still):
  report both and confirm they do not change.
- Note: concurrent reads alongside rover-uvicorn's battery read are fine;
  single transactions only.

## 4. GPS feed

- Sample `$XDG_RUNTIME_DIR/rover-gps-status.json` (fallback `/tmp`) every
  0.1 s for 60 s: record mtime, lat, lon, hacc_m, fix, sats, speed_mps,
  cog_deg, cog_acc_deg. Report: how often the file is rewritten, how often
  lat/lon actually CHANGE (the effective solution rate), and the fix/hAcc
  you see now (indoors is fine; say so).
- `journalctl --user -u rover-ntrip --since '120 days ago' --no-pager` :
  count "carrier solution" transitions and the "status:" lines; estimate the
  share of time in FIXED vs FLOAT vs none when outdoors, and the longest
  FIXED stretch. Note any receiver firmware/version lines if present.
- Does any script set the receiver navigation rate (CFG-RATE)? grep the repo
  and `~` for `CFG-RATE`, `0x30210001`, `configure_f9r`.

## 5. Data from previous attempts (Claude Mac suspects there is more)

- Nov 2025 "Rover PurePursuit" era: `find / -xdev \( -name 'wps_*.csv' -o -name 'drive.py' -o -name 'rover_lite.py' -o -name 'mission_planner.py' -o -name '*.log' -path '*rover*' \) 2>/dev/null`; also `~/paths/*.yaml` from the ROS patrol era. Copy small CSV/YAML/JSON/log files (not code, not anything over 1 MB) into `docs/rover-state-2026-10-31/prev-attempts/`.
- The July 19 software-in-the-loop sim for `app/nav.py` lived in a Claude
  scratchpad (`/tmp/claude-*/` or similar) and in session transcripts under
  `~/.claude/projects/`. If the script still exists, copy it to
  `tests/sim_nav_july2026.py` verbatim and note its three scenario results.
- Any field notes, photos' filenames, or console logs from the Nov 2025 runs
  or the July 19 sidewalk outing. Was `app/nav.py` ever started outdoors?
  `journalctl --user -u rover-uvicorn --since '120 days ago' | grep -iE 'nav|follow|acquiring|aligning|tracking|fault'` and summarize.

## 6. Your own read

- Which heading source would you trust on this rover and why?
- Anything you know about why the July follower never got its field run.
- Top three risks you see for a hands-off sidewalk 1 -> 4 -> 1 x3 run.

## Deliverable

1. Raw outputs as small text files under `docs/rover-state-2026-10-31/`
   (one file per section, plus `prev-attempts/`).
2. `handoff-claude-rover.txt` at the repo root: a concise summary with the
   answers to sections 1 to 6, each fact tagged with the command that
   produced it, and a short "what I could not determine" list.
3. `git add docs/rover-state-2026-10-31 handoff-claude-rover.txt tests/sim_nav_july2026.py` (if it exists), commit as
   `docs: rover recon for the GPS waypoint plan (read-only)`, push to
   `origin simplify`. Then paste the handoff summary in your reply.

No em dashes in anything you write; use commas, colons or parentheses.
