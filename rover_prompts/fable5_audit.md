# Fable 5 Audit Prompt (run on rover1)

You are running as a code-review agent on `rover1` — a Raspberry Pi 5 (Ubuntu 24.04, ROS 2 Jazzy) that is the brain of a mecanum-drive pack robot. This project (`rover1` / `rover2`, GitHub: `MeckMan2025/rover1`) is a personal pack robot that follows a person using YOLOv8 vision on a Hailo-8L accelerator, with RTK GPS (ZED-F9R), BerryIMU v3, and a web dashboard.

**Your task: perform a read-only code review / audit of the checkout on this machine. Do NOT modify files, commit, push, or change any hardware/system state. Investigate only.**

## Steps

1. **Locate & orient.** Find the repo checkout (likely `~/rover1`). Run `git rev-parse --abbrev-ref HEAD` and `git log --oneline -5` and report the branch + latest commit so it can be confirmed you're reviewing current code. List the top-level layout.

2. **Scope the review** to the actual rover software (prioritize in this order): `brain_launcher.py`, the `rover2_*` packages (`rover2_bringup`, `rover2_vision`, `rover2_hardware`, `rover2_dashboard`), then `rover1_*` packages, then the standalone scripts (`probe_*.py`, `verify_encoders.py`, `map_encoders.py`, GNSS tools). Skip PDFs, vendor archives, and docs.

3. **For each area, look for real defects** — not style nits. Focus on:
   - Correctness bugs (logic errors, wrong math in drive/kinematics or Kalman/GPS, off-by-one, race conditions, unhandled exceptions on the hot path)
   - Safety issues specific to a moving robot (teleop override gaps, person-loss handling, missing e-stop/timeout, motors not stopped on crash/exit)
   - Hardware/IO robustness (serial/I2C error handling, blocking calls, resource leaks, hardcoded device paths)
   - Concurrency and process-lifecycle issues in `brain_launcher.py`

4. **Ground every finding in the real code** — cite `file:line` and quote the relevant snippet. Do not report anything you haven't actually opened and read. If you suspect something but can't confirm, mark it "unverified" and say what's needed to confirm.

5. **Cross-check** against existing notes if present (`bug_log.md`, `rover2_audit_findings.md`, `refactor1.md`) and flag which of your findings are already known vs. new.

## Deliverable — report back in this exact format

```
BRANCH/COMMIT: <branch> @ <short-sha>
FILES REVIEWED: <count> — <list>

FINDINGS (most severe first):
[#1] SEVERITY: critical|high|medium|low  | NEW|KNOWN
  FILE: path/to/file.py:LINE
  ISSUE: <one sentence>
  EVIDENCE: <short quoted snippet>
  FAILURE MODE: <concrete scenario where this bites>
  FIX: <specific suggested change — description only, do not apply it>

... (repeat)

SUMMARY: <2-3 sentences: overall health, biggest risk, what you'd fix first>
UNVERIFIED / NEEDS FOLLOW-UP: <anything you couldn't confirm>
```

Keep it to the highest-signal findings (roughly 5–15). Do not make changes — this is review only. Report only what you verified in the actual files on this machine.
