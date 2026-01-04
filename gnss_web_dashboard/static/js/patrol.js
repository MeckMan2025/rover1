// patrol.js - Waypoint recording and patrol playback (Simplified UI)
class PatrolController {
    constructor(dashboard) {
        this.dashboard = dashboard;
        this.currentState = 'idle';
        this.pendingCallback = null;
        this.paths = [];
        this.selectedPathHasGps = false;
    }

    sendCommand(cmd) {
        this.dashboard.sendCommand(cmd);
    }

    handleResponse(data) {
        console.log('Patrol response:', data);
        if (data.paths !== undefined) {
            // Response from list_paths
            this.updatePathList(data.paths);
        } else if (data.image !== undefined) {
            // Response from get_map_image
            this.showMapPreview(data.image);
        } else if (data.pointcloud !== undefined) {
            // Response from get_pointcloud
            pointcloudViewer.hideLoading();
            if (data.success) {
                pointcloudViewer.show();
                pointcloudViewer.loadPointCloud(data.pointcloud);
            } else {
                alert(data.message || 'Failed to load point cloud');
            }
        } else if (!data.success) {
            // Error response
            pointcloudViewer.hideLoading();
            alert(data.message || 'Operation failed');
        }
    }

    updateStatus(status) {
        this.currentState = status.state;

        // Update status badge
        const badge = document.getElementById('patrolStatusBadge');
        const stateClass = `patrol-${status.state.replace('_', '-')}`;
        badge.className = `patrol-status-badge ${stateClass}`;

        let stateText = status.state.toUpperCase().replace('_', ' ');
        if (status.state === 'recording') {
            // Show GPS status during recording
            const gpsStatus = status.gps_fix_available ? 'GPS' : 'NO GPS';
            const gpsClass = status.gps_fix_available ? 'gps-ok' : 'gps-no';
            stateText = `RECORDING (${status.recorded_waypoint_count} pts)`;
        }
        badge.textContent = stateText;

        // Update progress display
        const progressEl = document.getElementById('patrolProgress');
        if (status.state === 'patrolling' || status.state === 'paused') {
            const loopText = status.total_loops > 0
                ? `Loop ${status.current_loop + 1} of ${status.total_loops}`
                : `Loop ${status.current_loop + 1} (continuous)`;
            progressEl.textContent = `Waypoint ${status.current_waypoint + 1} of ${status.total_waypoints} | ${loopText}`;
            progressEl.style.display = 'block';
        } else {
            progressEl.style.display = 'none';
        }

        // Update error display
        const errorEl = document.getElementById('patrolError');
        if (status.error_message) {
            errorEl.textContent = status.error_message;
            errorEl.style.display = 'block';
        } else {
            errorEl.style.display = 'none';
        }

        // Update recording indicator with GPS feedback
        const recordingIndicator = document.getElementById('recordingIndicator');
        const recordingCount = document.getElementById('recordingCount');
        if (status.state === 'recording') {
            recordingIndicator.style.display = 'flex';
            const gpsText = status.gps_fix_available
                ? `<span class="gps-status gps-ok">GPS: YES (${status.gps_waypoint_count}/${status.recorded_waypoint_count})</span>`
                : '<span class="gps-status gps-no">GPS: NO</span>';
            recordingCount.innerHTML = `Recording: ${status.recorded_waypoint_count} waypoints ${gpsText}`;
        } else {
            recordingIndicator.style.display = 'none';
        }

        // Update button states
        this.updateButtonStates(status.state);

        // Show/hide save dialog
        const saveDialog = document.getElementById('saveDialog');
        if (status.state === 'pending_save') {
            saveDialog.classList.add('active');
        } else {
            saveDialog.classList.remove('active');
        }
    }

    updateButtonStates(state) {
        const btnStartRecord = document.getElementById('btnStartRecord');
        const btnStopRecord = document.getElementById('btnStopRecord');
        const btnStartPatrol = document.getElementById('btnStartPatrol');
        const btnPausePatrol = document.getElementById('btnPausePatrol');
        const btnResumePatrol = document.getElementById('btnResumePatrol');
        const btnStopPatrol = document.getElementById('btnStopPatrol');

        // Reset all buttons
        btnStartRecord.disabled = false;
        btnStopRecord.disabled = true;
        btnStartPatrol.disabled = false;
        btnPausePatrol.disabled = true;
        btnResumePatrol.disabled = true;
        btnStopPatrol.disabled = true;

        switch (state) {
            case 'idle':
                // Default state - can start recording or patrol
                break;
            case 'recording':
                btnStartRecord.disabled = true;
                btnStopRecord.disabled = false;
                btnStartPatrol.disabled = true;
                break;
            case 'pending_save':
                btnStartRecord.disabled = true;
                btnStopRecord.disabled = true;
                btnStartPatrol.disabled = true;
                break;
            case 'patrolling':
                btnStartRecord.disabled = true;
                btnStartPatrol.disabled = true;
                btnPausePatrol.disabled = false;
                btnStopPatrol.disabled = false;
                break;
            case 'paused':
                btnStartRecord.disabled = true;
                btnStartPatrol.disabled = true;
                btnResumePatrol.disabled = false;
                btnStopPatrol.disabled = false;
                break;
            case 'error':
                // Can try again
                break;
        }
    }

    // Recording controls
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
        // Refresh paths after saving
        setTimeout(() => this.refreshPaths(), 500);
    }

    discardRecording() {
        this.sendCommand({ action: 'discard_recording' });
    }

    // Path management
    refreshPaths() {
        this.sendCommand({ action: 'list_paths' });
    }

    updatePathList(paths) {
        this.paths = paths;
        const select = document.getElementById('pathSelect');
        select.innerHTML = '<option value="">-- Select a path --</option>';

        paths.forEach(path => {
            const option = document.createElement('option');
            option.value = path.name;
            // Show GPS indicator in path list
            const gpsIndicator = path.has_gps ? ' [GPS]' : '';
            option.textContent = `${path.name} (${path.waypoint_count} pts${gpsIndicator}, ${path.recorded_date})`;
            option.dataset.hasGps = path.has_gps;
            option.dataset.gpsCount = path.gps_waypoint_count;
            select.appendChild(option);
        });
    }

    onPathSelect() {
        const pathSelect = document.getElementById('pathSelect');
        const pathName = pathSelect.value;
        const autoModeSelect = document.getElementById('autoMode');
        const gpsIndicator = document.getElementById('gpsIndicator');
        const btn3D = document.getElementById('btnView3D');

        if (pathName) {
            // Get GPS info from selected option
            const selectedOption = pathSelect.options[pathSelect.selectedIndex];
            this.selectedPathHasGps = selectedOption.dataset.hasGps === 'true';

            // Auto-select mode based on GPS availability
            if (this.selectedPathHasGps) {
                autoModeSelect.value = 'gps';
                gpsIndicator.textContent = 'GPS mode available';
                gpsIndicator.className = 'gps-indicator gps-available';
                autoModeSelect.disabled = false;
            } else {
                autoModeSelect.value = 'odometry';
                gpsIndicator.textContent = 'Indoor mode only (no GPS data)';
                gpsIndicator.className = 'gps-indicator gps-unavailable';
                // Only show odometry option
                autoModeSelect.disabled = true;
            }

            this.sendCommand({ action: 'get_map_image', path_name: pathName });
            btn3D.disabled = false;
        } else {
            this.clearMapPreview();
            btn3D.disabled = true;
            gpsIndicator.textContent = '';
            gpsIndicator.className = 'gps-indicator';
            autoModeSelect.disabled = false;
        }
    }

    view3DCloud() {
        const pathName = document.getElementById('pathSelect').value;
        if (!pathName) {
            alert('Please select a path first');
            return;
        }
        pointcloudViewer.showLoading();
        this.sendCommand({ action: 'get_pointcloud', path_name: pathName });
    }

    showMapPreview(base64Image) {
        const preview = document.getElementById('mapPreview');
        if (base64Image) {
            preview.innerHTML = `<img src="data:image/png;base64,${base64Image}" alt="Path Map">`;
        } else {
            this.clearMapPreview();
        }
    }

    clearMapPreview() {
        const preview = document.getElementById('mapPreview');
        preview.innerHTML = '<div class="map-placeholder">Select a path to preview map</div>';
    }

    toggleAdvanced() {
        const advancedOptions = document.getElementById('advancedOptions');
        const toggleBtn = document.getElementById('btnToggleAdvanced');
        if (advancedOptions.style.display === 'none') {
            advancedOptions.style.display = 'block';
            toggleBtn.textContent = 'Hide Options';
        } else {
            advancedOptions.style.display = 'none';
            toggleBtn.textContent = 'More Options';
        }
    }

    // Patrol controls
    startPatrol() {
        const pathName = document.getElementById('pathSelect').value;
        if (!pathName) {
            alert('Please select a path first');
            return;
        }

        const autoMode = document.getElementById('autoMode').value;
        const loopCount = parseInt(document.getElementById('loopCount').value);
        const speedPercent = parseInt(document.getElementById('speedSlider').value) / 100;
        const reverseMode = document.getElementById('reverseMode').checked;

        // Store current mode for pause/resume/stop
        this.currentAutoMode = autoMode;

        this.sendCommand({
            action: 'start_patrol',
            auto_mode: autoMode,
            path_name: pathName,
            loop_count: loopCount,
            speed_percent: speedPercent,
            reverse_mode: reverseMode
        });
    }

    pausePatrol() {
        this.sendCommand({
            action: 'pause_patrol',
            auto_mode: this.currentAutoMode || 'odometry'
        });
    }

    resumePatrol() {
        this.sendCommand({
            action: 'resume_patrol',
            auto_mode: this.currentAutoMode || 'odometry'
        });
    }

    stopPatrol() {
        this.sendCommand({
            action: 'stop_patrol',
            auto_mode: this.currentAutoMode || 'odometry'
        });
    }

    updateSpeedLabel() {
        const value = document.getElementById('speedSlider').value;
        document.getElementById('speedValue').textContent = value;
    }
}


window.PatrolController = PatrolController;
