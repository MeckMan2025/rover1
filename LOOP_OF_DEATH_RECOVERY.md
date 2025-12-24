# 🚨 Rover1: Loop of Death Recovery Plan

If the rover is caught in a reboot/crash loop due to the autonomous startup service, follow this guide to regain control.

---

## 1. Symptoms
- The rover keeps restarting (logs show `Service restarted`).
- Foxglove connects and then immediately disconnects.
- `systemctl status rover1.service` shows `Active: failed (Result: start-limit-hit)`.

---

## 2. Immediate Recovery (The "Kill Switch")
Every time the rover boots, there is a **15-second mandatory delay** before code is pulled or launched. You must use this window to act.

1.  **SSH into the rover**:
    `ssh andrewmeckley@rover1.local`
2.  **Activate the Bypass File**:
    ```bash
    touch ~/ros2_ws/src/rover1/STOP_ROVER
    ```
    *The startup script will now see this file and exit immediately on every boot, giving you a safe environment to work.*

---

## 3. Diagnosis (Finding the Bug)
Once the rover is stable, look at the "Black Box" logs to see what caused the crash:

- **View full crash history**:
  ```bash
  journalctl -u rover1.service -n 100 --no-pager
  ```
- **Watch logs in real-time** (if you decide to try launching again):
  ```bash
  journalctl -u rover1.service -f
  ```

---

## 4. Manual Control
If you want to stop the service entirely while you debug manually:
```bash
sudo systemctl stop rover1.service
```

---

## 5. Resuming Normal Operations
Once the bug is fixed and committed to Git:

1.  **Pull the fix manually**:
    ```bash
    cd ~/ros2_ws/src/rover1
    git pull
    ```
2.  **Remove the Kill Switch**:
    ```bash
    rm ~/ros2_ws/src/rover1/STOP_ROVER
    ```
3.  **Restart the service**:
    ```bash
    sudo systemctl start rover1.service
    ```

---

## 🛡️ Safety Systems Installed
- **StartLimitIntervalSec=300**: Analyzes 5-minute windows.
- **StartLimitBurst=5**: If the rover crashes 5 times in 5 minutes, it **stops trying** for 5 minutes.
- **Git Safety**: If `git pull` fails due to a merge conflict, the rover skips the update and runs the last known good code.
