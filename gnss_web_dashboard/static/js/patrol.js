// patrol.js - Minimal Teach & Repeat UI
// 6 buttons: Record, Stop Recording, Save, Discard, Start Patrol, Stop

class PatrolController {
    constructor(dashboard) {
        this.dashboard = dashboard;
        this.paths = [];
        this.selectedPathHasGps = false;
        this.isRecording = false;
        this.isPatrolling = false;
    }

    sendCommand(cmd) {
        this.dashboard.sendCommand(cmd);
    }

    handleResponse(data) {
        if (data.paths !== undefined) {
            this.updatePathList(data.paths);
        } else if (!data.success && data.message) {
            alert(data.message);
        }
    }

    updateStatus(status) {
        const state = status.state;

        // RECORD MODE UI
        const btnRecord = document.getElementById('btnRecord');
        const recordingStatus = document.getElementById('recordingStatus');
        const saveDialog = document.getElementById('saveDialog');
        const waypointCount = document.getElementById('waypointCount');
        const gpsStatus = document.getElementById('gpsStatus');

        if (state === 'recording') {
            this.isRecording = true;
            btnRecord.style.display = 'none';
            recordingStatus.style.display = 'block';
            saveDialog.style.display = 'none';
            waypointCount.textContent = status.recorded_waypoint_count;

            // GPS status: checkmark or X
            if (status.gps_fix_available) {
                gpsStatus.innerHTML = 'GPS: <span class="gps-yes">&#10003;</span>';
                gpsStatus.className = 'gps-check gps-available';
            } else {
                gpsStatus.innerHTML = 'GPS: <span class="gps-no">&#10007;</span>';
                gpsStatus.className = 'gps-check gps-unavailable';
            }
        } else if (state === 'pending_save') {
            this.isRecording = false;
            btnRecord.style.display = 'none';
            recordingStatus.style.display = 'none';
            saveDialog.style.display = 'block';
        } else if (state === 'idle' && !this.isPatrolling) {
            this.isRecording = false;
            btnRecord.style.display = 'block';
            recordingStatus.style.display = 'none';
            saveDialog.style.display = 'none';
        }

        // PATROL MODE UI
        const btnStartPatrol = document.getElementById('btnStartPatrol');
        const patrolStatus = document.getElementById('patrolStatus');
        const patrolProgress = document.getElementById('patrolProgress');
        const patrolError = document.getElementById('patrolError');

        if (state === 'patrolling' || state === 'paused') {
            this.isPatrolling = true;
            btnStartPatrol.style.display = 'none';
            patrolStatus.style.display = 'block';

            // Show progress: "5/12 (Loop 3)"
            const wp = status.current_waypoint + 1;
            const total = status.total_waypoints;
            const loop = status.current_loop + 1;
            patrolProgress.textContent = `Patrolling: ${wp}/${total} (Loop ${loop})`;

            if (state === 'paused') {
                patrolProgress.textContent += ' [PAUSED]';
            }
        } else if (state === 'idle' || state === 'error') {
            this.isPatrolling = false;
            btnStartPatrol.style.display = 'block';
            patrolStatus.style.display = 'none';
        }

        // Error display
        if (status.error_message) {
            patrolError.textContent = status.error_message;
            patrolError.style.display = 'block';
        } else {
            patrolError.style.display = 'none';
        }

        // Disable patrol controls during recording
        const pathSelect = document.getElementById('pathSelect');
        pathSelect.disabled = this.isRecording;
        btnStartPatrol.disabled = this.isRecording || !pathSelect.value;
    }

    // === RECORDING ===

    startRecording() {
        this.sendCommand({ action: 'start_recording' });
    }

    stopRecording() {
        this.sendCommand({ action: 'stop_recording' });
    }

    savePath() {
        const pathName = document.getElementById('pathNameInput').value.trim();
        if (!pathName) {
            alert('Please enter a path name');
            return;
        }
        this.sendCommand({ action: 'save_path', path_name: pathName });
        document.getElementById('pathNameInput').value = '';
        setTimeout(() => this.refreshPaths(), 500);
    }

    discardRecording() {
        this.sendCommand({ action: 'discard_recording' });
    }

    // === PATH MANAGEMENT ===

    refreshPaths() {
        this.sendCommand({ action: 'list_paths' });
    }

    updatePathList(paths) {
        this.paths = paths;
        const select = document.getElementById('pathSelect');
        select.innerHTML = '<option value="">Select path...</option>';

        paths.forEach(path => {
            const option = document.createElement('option');
            option.value = path.name;
            const gpsTag = path.has_gps ? ' [GPS]' : '';
            option.textContent = `${path.name} (${path.waypoint_count} pts${gpsTag})`;
            option.dataset.hasGps = path.has_gps;
            select.appendChild(option);
        });
    }

    onPathSelect() {
        const pathSelect = document.getElementById('pathSelect');
        const btnStartPatrol = document.getElementById('btnStartPatrol');

        if (pathSelect.value) {
            const selectedOption = pathSelect.options[pathSelect.selectedIndex];
            this.selectedPathHasGps = selectedOption.dataset.hasGps === 'true';
            btnStartPatrol.disabled = false;
        } else {
            this.selectedPathHasGps = false;
            btnStartPatrol.disabled = true;
        }
    }

    // === PATROL ===

    startPatrol() {
        const pathName = document.getElementById('pathSelect').value;
        if (!pathName) {
            alert('Please select a path first');
            return;
        }

        // Auto-detect mode: GPS if path has GPS waypoints, else Odometry
        const autoMode = this.selectedPathHasGps ? 'gps' : 'odometry';

        this.sendCommand({
            action: 'start_patrol',
            auto_mode: autoMode,
            path_name: pathName,
            loop_count: 0,        // Always infinite
            speed_percent: 1.0,   // Always 100%
            reverse_mode: false   // Always forward
        });

        this.currentAutoMode = autoMode;
    }

    stopPatrol() {
        this.sendCommand({
            action: 'stop_patrol',
            auto_mode: this.currentAutoMode || 'odometry'
        });
    }
}

window.PatrolController = PatrolController;
