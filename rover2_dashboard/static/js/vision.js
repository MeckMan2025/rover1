// vision.js - Person follower controller for Rover2
class PersonFollowerController {
    constructor(dashboard) {
        this.dashboard = dashboard;
        this.detectionEnabled = false;
        this.followingEnabled = false;
        this.currentStatus = 'idle';
        this.usingAiFeed = false;
        this.nodeRunning = false;
    }

    toggleDetection(enabled) {
        if (enabled) {
            this.dashboard.sendCommand({ action: 'person_detection_enable' });
        } else {
            this.dashboard.sendCommand({ action: 'person_detection_disable' });
        }
    }

    enable() {
        this.dashboard.sendCommand({ action: 'person_follow_enable' });
    }

    disable() {
        this.dashboard.sendCommand({ action: 'person_follow_disable' });
    }

    reset() {
        this.dashboard.sendCommand({ action: 'person_follow_reset' });
    }

    updateStatus(status, nodeRunning = true) {
        this.currentStatus = status;
        this.nodeRunning = nodeRunning;

        // If node isn't running, show OFF state
        if (!nodeRunning) {
            this.followingEnabled = false;
            this.detectionEnabled = false;

            const toggle = document.getElementById('personDetectionToggle');
            if (toggle) toggle.checked = false;

            const btnStart = document.getElementById('btnPersonFollowStart');
            const btnStop = document.getElementById('btnPersonFollowStop');
            if (btnStart) btnStart.disabled = true;
            if (btnStop) btnStop.disabled = true;

            const statusEl = document.getElementById('personFollowerStatus');
            if (statusEl) {
                statusEl.className = 'quick-status idle';
                statusEl.textContent = 'OFF';
            }
            return;
        }

        // Following is active for searching, following, lost_target, recovery_scan states
        this.followingEnabled = ['searching', 'following', 'lost_target', 'recovery_scan'].includes(status);
        // Detection is on if not idle or teleop_override
        const detectionOn = (status !== 'idle' && status !== 'teleop_override');

        // Update toggle state
        const toggle = document.getElementById('personDetectionToggle');
        if (toggle && toggle.checked !== detectionOn) {
            toggle.checked = detectionOn;
        }
        this.detectionEnabled = detectionOn;

        // Update button states
        const btnStart = document.getElementById('btnPersonFollowStart');
        const btnStop = document.getElementById('btnPersonFollowStop');
        if (btnStart) btnStart.disabled = !this.detectionEnabled || this.followingEnabled;
        if (btnStop) btnStop.disabled = !this.followingEnabled;

        // Status text with friendly names
        const statusText = {
            'idle': 'IDLE',
            'detecting': 'DETECTING',
            'searching': 'SEARCHING...',
            'following': 'FOLLOWING',
            'lost_target': 'LOST',
            'teleop_override': 'OVERRIDE',
            'recovery_scan': 'SCANNING...'
        };
        const statusClass = status.replace('_', '-');

        // Update status badge
        const statusEl = document.getElementById('personFollowerStatus');
        if (statusEl) {
            statusEl.className = 'quick-status ' + statusClass;
            statusEl.textContent = statusText[status] || status.toUpperCase();
        }
    }

    updateFeedIndicator(usingAiFeed) {
        // Feed indicator tracking
        this.usingAiFeed = usingAiFeed;
    }

    // Sync state from server (called when receiving data)
    syncFromServer(detectionEnabled, usingAiFeed, nodeRunning = true) {
        this.detectionEnabled = detectionEnabled;
        this.usingAiFeed = usingAiFeed;
        this.nodeRunning = nodeRunning;

        // If node isn't running, ensure toggle is off
        if (!nodeRunning) {
            const toggle = document.getElementById('personDetectionToggle');
            if (toggle) toggle.checked = false;
            const btnStart = document.getElementById('btnPersonFollowStart');
            if (btnStart) btnStart.disabled = true;
            return;
        }

        // Update toggle without triggering onchange
        const toggle = document.getElementById('personDetectionToggle');
        if (toggle && toggle.checked !== detectionEnabled) {
            toggle.checked = detectionEnabled;
        }

        // Update start button based on detection state
        const btnStart = document.getElementById('btnPersonFollowStart');
        if (btnStart) btnStart.disabled = !detectionEnabled || this.followingEnabled;
    }
}