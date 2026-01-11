// dashboard.js - Core WebSocket and state management
class Rover1Dashboard {
    constructor() {
        this.websocket = null;
        this.reconnectInterval = 5000;
        this.lastUpdateTime = null;
        this.patrolController = null;

        // Battery voltage staleness tracking - prevents flickering to '--' during brief data gaps
        this.lastValidBatteryVoltage = null;
        this.lastValidBatteryTime = null;
        this.batteryStaleTimeout = 5000; // Show '--' only if no valid data for 5 seconds

        this.connect();

        // Update "last updated" every second
        setInterval(() => this.updateLastUpdateDisplay(), 1000);
    }

    setPatrolController(controller) {
        this.patrolController = controller;
    }

    setTeleopController(controller) {
        this.teleopController = controller;
    }

    connect() {
        const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
        const wsUrl = `${protocol}//${window.location.hostname}:8081`;

        console.log('Connecting to:', wsUrl);
        this.websocket = new WebSocket(wsUrl);

        this.websocket.onopen = () => {
            console.log('WebSocket connected');
            this.updateConnectionStatus(true);
            // Refresh paths when connected
            if (this.patrolController) {
                this.patrolController.refreshPaths();
            }
        };

        this.websocket.onmessage = (event) => {
            const data = JSON.parse(event.data);
            // Check if this is a command response
            if (data.success !== undefined && !data.patrol) {
                // This is a response to a command
                if (this.patrolController) {
                    this.patrolController.handleResponse(data);
                }
            } else {
                // This is a status broadcast
                this.updateDashboard(data);
            }
        };

        this.websocket.onclose = () => {
            console.log('WebSocket disconnected');
            this.updateConnectionStatus(false);
            setTimeout(() => this.connect(), this.reconnectInterval);
        };

        this.websocket.onerror = (error) => {
            console.error('WebSocket error:', error);
            this.updateConnectionStatus(false);
        };
    }

    sendCommand(cmd) {
        if (this.websocket && this.websocket.readyState === WebSocket.OPEN) {
            this.websocket.send(JSON.stringify(cmd));
        } else {
            console.error('WebSocket not connected');
        }
    }

    updateConnectionStatus(connected) {
        const statusEl = document.getElementById('connectionStatus');
        if (connected) {
            statusEl.textContent = '● Connected';
            statusEl.className = 'connection-status connected';
        } else {
            statusEl.textContent = '● Disconnected';
            statusEl.className = 'connection-status disconnected';
        }
    }

    updateDashboard(data) {
        this.lastUpdateTime = Date.now();

        // RTK Status
        if (data.rtk_state !== undefined) {
            this.updateRTKStatus(data.rtk_state, data.h_acc_m);
        }

        // Satellites
        if (data.sat_visible !== undefined) {
            document.getElementById('satVisible').textContent = data.sat_visible || '--';
            document.getElementById('satUsed').textContent = data.sat_used || '--';
        }

        // NTRIP Status
        if (data.ntrip_connected !== undefined) {
            this.updateNTRIPStatus(data.ntrip_connected, data.rtcm_msgs_per_sec, data.corr_age_s);
        }

        // Data Statistics
        if (data.rtcm_msgs_total !== undefined) {
            document.getElementById('totalMessages').textContent =
                data.rtcm_msgs_total ? data.rtcm_msgs_total.toLocaleString() : '--';
            document.getElementById('dataRate').textContent =
                data.rtcm_bytes_per_sec ? Math.round(data.rtcm_bytes_per_sec) : '--';
        }

        // Video Feed
        if (data.camera_image) {
            this.updateVideoFeed(data.camera_image);
        }

        // HUD Overlays
        this.updateHUD(data);

        // Battery Status - with staleness protection to prevent flickering
        if (data.battery_voltage !== undefined && data.battery_voltage !== null && data.battery_voltage > 0) {
            // Valid reading - update and cache
            this.lastValidBatteryVoltage = data.battery_voltage;
            this.lastValidBatteryTime = Date.now();
            this.updateBatteryStatus(data.battery_voltage);
        } else if (this.lastValidBatteryVoltage !== null) {
            // Invalid/null reading - check staleness before showing '--'
            if (Date.now() - this.lastValidBatteryTime < this.batteryStaleTimeout) {
                // Keep showing last valid value (data not stale yet)
                this.updateBatteryStatus(this.lastValidBatteryVoltage);
            } else {
                // Data is stale, show '--'
                this.updateBatteryStatus(null);
            }
        }

        // Power Supply Status (USB-C PD)
        if (data.power_status) {
            this.updatePowerStatus(data.power_status);
        }

        // Patrol Status
        if (data.patrol) {
            this.updatePatrolStatus(data.patrol);
        }

        // Teleop Speed sync
        if (data.teleop_speed !== undefined && this.teleopController) {
            this.teleopController.syncFromServer(data.teleop_speed);
        }

        // Dog Follower Status and Running state
        if (dogFollower) {
            const nodeRunning = data.dog_follower_running || false;
            if (data.dog_follower_status !== undefined) {
                dogFollower.updateStatus(data.dog_follower_status, nodeRunning);
            }
            dogFollower.syncFromServer(
                data.detection_enabled || false,
                data.using_ai_feed || false,
                nodeRunning
            );
        }

        // Shoe Follower Status and Running state
        if (shoeFollower) {
            const nodeRunning = data.shoe_follower_running || false;
            if (data.shoe_follower_status !== undefined) {
                shoeFollower.updateStatus(data.shoe_follower_status, nodeRunning);
            }
            shoeFollower.syncFromServer(
                data.shoe_detection_enabled || false,
                nodeRunning
            );
        }

        // Map location
        if (data.latitude !== undefined && data.longitude !== undefined) {
            this.updateMapLocation(data.latitude, data.longitude, data.altitude, data.rtk_state);
        }
    }

    updatePatrolStatus(patrol) {
        if (this.patrolController) {
            this.patrolController.updateStatus(patrol);
        }
    }

    updateMapLocation(lat, lon, alt, rtkState) {
        if (mapController) {
            mapController.updatePosition(lat, lon, alt, rtkState);
        }
    }

    updateVideoFeed(base64Data) {
        const img = document.getElementById('videoFeed');
        const placeholder = document.getElementById('videoPlaceholder');
        
        if (base64Data) {
            img.src = `data:image/jpeg;base64,${base64Data}`;
            img.style.display = 'block';
            placeholder.style.display = 'none';
        }
    }

    updateHUD(data) {
        const rtk = document.getElementById('hudRtk');
        const sat = document.getElementById('hudSat');
        const volt = document.getElementById('hudVolt');
        const wifi = document.getElementById('hudWifi');

        if (data.rtk_state) {
            rtk.textContent = `RTK: ${data.rtk_state}`;
            rtk.style.color = data.rtk_state === 'FIXED' ? '#00ff88' :
                             (data.rtk_state === 'FLOAT' ? '#0088ff' : '#ffc107');
        }

        if (data.sat_used !== undefined) {
            sat.textContent = `SAT: ${data.sat_used}`;
        }

        // HUD voltage uses same staleness-protected value as battery status
        if (data.battery_voltage !== undefined && data.battery_voltage !== null && data.battery_voltage > 0) {
            volt.textContent = `VOLT: ${data.battery_voltage.toFixed(1)} V`;
        } else if (this.lastValidBatteryVoltage !== null) {
            // Use cached value if not stale
            if (Date.now() - this.lastValidBatteryTime < this.batteryStaleTimeout) {
                volt.textContent = `VOLT: ${this.lastValidBatteryVoltage.toFixed(1)} V`;
            } else {
                volt.textContent = 'VOLT: -- V';
            }
        }

        // WiFi signal strength with color coding
        if (data.wifi_signal !== undefined && data.wifi_signal !== null) {
            wifi.textContent = `WIFI: ${data.wifi_signal}%`;
            // Color code: green >60%, yellow 30-60%, red <30%
            if (data.wifi_signal >= 60) {
                wifi.style.color = '#00ff88';
            } else if (data.wifi_signal >= 30) {
                wifi.style.color = '#ffc107';
            } else {
                wifi.style.color = '#ff4444';
            }
        } else {
            wifi.textContent = 'WIFI: --%';
            wifi.style.color = '#888888';
        }
    }

    updateRTKStatus(state, accuracy) {
        const badge = document.getElementById('rtkBadge');
        const accuracyEl = document.getElementById('rtkAccuracy');
        const fill = document.getElementById('accuracyFill');
        
        // Update badge
        badge.textContent = state || 'NO FIX';
        badge.className = `rtk-badge rtk-${(state || 'no-fix').toLowerCase().replace('_', '-')}`;
        
        // Update accuracy
        if (accuracy && accuracy > 0) {
            const accMeters = accuracy.toFixed(3);
            const accCm = (accuracy * 100).toFixed(1);
            accuracyEl.textContent = `Accuracy: ${accCm}cm`;
            
            // Update accuracy bar
            let fillPercent, fillClass;
            if (accuracy <= 0.02) {
                fillPercent = 100; fillClass = 'accuracy-excellent';
            } else if (accuracy <= 0.1) {
                fillPercent = 80; fillClass = 'accuracy-good';
            } else if (accuracy <= 0.5) {
                fillPercent = 60; fillClass = 'accuracy-fair';
            } else {
                fillPercent = 30; fillClass = 'accuracy-poor';
            }
            
            fill.style.width = fillPercent + '%';
            fill.className = `accuracy-fill ${fillClass}`;
        } else {
            accuracyEl.textContent = 'Accuracy: --';
            fill.style.width = '0%';
            fill.className = 'accuracy-fill accuracy-poor';
        }
    }
    
    updateNTRIPStatus(connected, rate, age) {
        const dot = document.getElementById('ntripDot');
        const status = document.getElementById('ntripStatus');
        const rateEl = document.getElementById('rtcmRate');
        const ageEl = document.getElementById('rtcmAge');

        if (connected) {
            dot.className = 'status-dot status-connected';
            status.textContent = 'CONNECTED';
        } else {
            dot.className = 'status-dot status-disconnected';
            status.textContent = 'DISCONNECTED';
        }

        rateEl.textContent = rate ? rate.toFixed(1) : '--';
        ageEl.textContent = age ? age.toFixed(1) : '--';
    }

    updateBatteryStatus(voltage) {
        const voltageEl = document.getElementById('batteryVoltage');
        const fill = document.getElementById('batteryFill');
        const statusEl = document.getElementById('batteryStatus');

        if (voltage && voltage > 0) {
            voltageEl.textContent = voltage.toFixed(2);

            // Battery percentage and status based on typical LiPo 3S (11.1V nominal)
            // Full: 12.6V, Empty: 9.9V (3.3V per cell cutoff)
            const minV = 9.9;
            const maxV = 12.6;
            const percent = Math.max(0, Math.min(100, ((voltage - minV) / (maxV - minV)) * 100));

            let fillClass, statusText, voltageColor;
            if (voltage >= 12.0) {
                fillClass = 'battery-full';
                statusText = 'FULL';
                voltageColor = '#00ff88';
            } else if (voltage >= 11.1) {
                fillClass = 'battery-good';
                statusText = 'GOOD';
                voltageColor = '#0088ff';
            } else if (voltage >= 10.5) {
                fillClass = 'battery-low';
                statusText = 'LOW';
                voltageColor = '#ffc107';
            } else {
                fillClass = 'battery-critical';
                statusText = 'CRITICAL';
                voltageColor = '#ff4444';
            }

            fill.style.width = percent + '%';
            fill.className = `battery-fill ${fillClass}`;
            statusEl.textContent = `${statusText} (${Math.round(percent)}%)`;
            voltageEl.style.color = voltageColor;
        } else {
            voltageEl.textContent = '--';
            voltageEl.style.color = 'rgba(255, 255, 255, 0.5)';
            fill.style.width = '0%';
            fill.className = 'battery-fill battery-good';
            statusEl.textContent = 'NO DATA';
        }
    }

    updatePowerStatus(powerStatus) {
        const iconEl = document.getElementById('powerIcon');
        const statusEl = document.getElementById('powerStatus');

        if (!powerStatus) {
            statusEl.textContent = 'USB-C: --';
            statusEl.className = 'power-status-text power-unknown';
            return;
        }

        const status = powerStatus.status;
        let colorClass, displayText;

        switch (status) {
            case 'OK':
                colorClass = 'power-ok';
                displayText = 'USB-C: OK';
                break;
            case 'RECOVERED':
                colorClass = 'power-warning';
                displayText = 'USB-C: RECOVERED';
                break;
            case 'THROTTLED':
                colorClass = 'power-warning';
                displayText = powerStatus.soft_temp_limit ? 'TEMP THROTTLED' : 'FREQ CAPPED';
                break;
            case 'RESTRICTED':
                colorClass = 'power-critical';
                displayText = powerStatus.under_voltage ? 'LOW VOLTAGE!' : 'THROTTLED!';
                break;
            default:
                colorClass = 'power-unknown';
                displayText = 'USB-C: UNKNOWN';
        }

        statusEl.textContent = displayText;
        statusEl.className = `power-status-text ${colorClass}`;
    }

    updateLastUpdateDisplay() {
        const updateEl = document.getElementById('lastUpdate');
        if (this.lastUpdateTime) {
            const secondsAgo = Math.floor((Date.now() - this.lastUpdateTime) / 1000);
            updateEl.textContent = `Last updated: ${secondsAgo}s ago`;
        } else {
            updateEl.textContent = 'Last updated: Never';
        }
    }
}

// Export for use by other modules
window.Rover1Dashboard = Rover1Dashboard;
