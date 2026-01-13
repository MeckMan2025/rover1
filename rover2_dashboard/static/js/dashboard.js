// dashboard.js - Core WebSocket and state management
class Rover1Dashboard {
    constructor() {
        this.websocket = null;
        this.reconnectInterval = 5000;
        this.lastUpdateTime = null;

        // Battery voltage staleness tracking - prevents flickering to '--' during brief data gaps
        this.lastValidBatteryVoltage = null;
        this.lastValidBatteryTime = null;
        this.batteryStaleTimeout = 5000; // Show '--' only if no valid data for 5 seconds

        this.connect();

        // Update "last updated" every second
        setInterval(() => this.updateLastUpdateDisplay(), 1000);
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
        };

        this.websocket.onmessage = (event) => {
            try {
                const data = JSON.parse(event.data);
                // This is a status broadcast - update dashboard
                this.updateDashboard(data);
            } catch (error) {
                console.error('Error processing WebSocket message:', error);
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
        if (!statusEl) return;
        
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
            const satVisEl = document.getElementById('satVisible');
            const satUsedEl = document.getElementById('satUsed');
            if (satVisEl) satVisEl.textContent = data.sat_visible || '--';
            if (satUsedEl) satUsedEl.textContent = data.sat_used || '--';
        }

        // NTRIP Status
        if (data.ntrip_connected !== undefined) {
            this.updateNTRIPStatus(data.ntrip_connected, data.rtcm_msgs_per_sec, data.corr_age_s);
        }

        // Data Statistics
        if (data.rtcm_msgs_total !== undefined) {
            const totalMsgEl = document.getElementById('totalMessages');
            const dataRateEl = document.getElementById('dataRate');
            if (totalMsgEl) {
                totalMsgEl.textContent = data.rtcm_msgs_total ? data.rtcm_msgs_total.toLocaleString() : '--';
            }
            if (dataRateEl) {
                dataRateEl.textContent =
                    data.rtcm_bytes_per_sec ? Math.round(data.rtcm_bytes_per_sec) : '--';
            }
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


        // Teleop Speed sync
        if (data.teleop_speed !== undefined && this.teleopController) {
            this.teleopController.syncFromServer(data.teleop_speed);
        }

        // Person Follower Status and Running state (rover2)
        if (personFollower) {
            const nodeRunning = data.person_follower_running || false;
            if (data.person_follower_status !== undefined) {
                personFollower.updateStatus(data.person_follower_status, nodeRunning);
            }
            personFollower.syncFromServer(
                data.person_detection_enabled || false,
                data.using_ai_feed || false,
                nodeRunning
            );
        }

        // Map location
        if (data.latitude !== undefined && data.longitude !== undefined) {
            this.updateMapLocation(data.latitude, data.longitude, data.altitude, data.rtk_state);
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
        
        // Guard against missing elements
        if (!img || !placeholder) {
            return;
        }
        
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
        const detection = document.getElementById('hudDetection');

        // Guard against missing HUD elements
        if (!rtk || !sat || !volt || !wifi || !detection) {
            return;
        }

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

        // Person Detection Status (rover2)
        this.updateDetectionHUD(data, detection);
    }

    updateDetectionHUD(data, detection) {
        const detectionEnabled = data.person_detection_enabled || false;
        const followerStatus = data.person_follower_status || 'idle';
        const nodeRunning = data.person_follower_running || false;

        if (!nodeRunning) {
            // Node not running
            detection.textContent = 'DETECT: OFF';
            detection.style.color = '#888888';
            return;
        }

        // Map status to display and colors
        if (!detectionEnabled || followerStatus === 'idle') {
            detection.textContent = 'DETECT: IDLE';
            detection.style.color = 'rgba(255, 255, 255, 0.6)';
        } else if (followerStatus === 'detecting') {
            detection.textContent = 'DETECT: ON';
            detection.style.color = '#9333ea';
        } else if (followerStatus === 'searching') {
            detection.textContent = 'DETECT: SEARCH';
            detection.style.color = '#ffc107';
        } else if (followerStatus === 'following') {
            detection.textContent = 'DETECT: FOLLOW';
            detection.style.color = '#00ff88';
        } else if (followerStatus === 'lost_target') {
            detection.textContent = 'DETECT: LOST';
            detection.style.color = '#ff9800';
        } else if (followerStatus === 'teleop_override') {
            detection.textContent = 'DETECT: OVERRIDE';
            detection.style.color = '#ff4444';
        } else if (followerStatus === 'recovery_scan') {
            detection.textContent = 'DETECT: SCAN';
            detection.style.color = '#ffc107';
        } else {
            detection.textContent = 'DETECT: UNKNOWN';
            detection.style.color = '#888888';
        }
    }

    updateRTKStatus(state, accuracy) {
        const badge = document.getElementById('rtkBadge');
        const accuracyEl = document.getElementById('rtkAccuracy');
        const fill = document.getElementById('accuracyFill');
        
        // Guard against missing elements
        if (!badge || !accuracyEl || !fill) {
            return;
        }
        
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

        // Guard against missing elements
        if (!dot || !status || !rateEl || !ageEl) {
            return;
        }

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
        // Update both old-style elements (if present) and new slim elements
        const voltageEl = document.getElementById('batteryVoltage');
        const fill = document.getElementById('batteryFill');
        const statusEl = document.getElementById('batteryStatus');
        
        // New slim elements
        const voltageSlimEl = document.getElementById('batteryVoltageSlim');
        const fillSlim = document.getElementById('batteryFillSlim');
        const statusSlimEl = document.getElementById('batteryStatusSlim');

        // Guard against missing elements - rover2 has both regular and slim battery elements
        if (!voltageEl && !fill && !statusEl && !voltageSlimEl && !fillSlim && !statusSlimEl) {
            return;
        }

        if (voltage && voltage > 0) {
            const voltageText = voltage.toFixed(2) + ' V';
            
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

            // Update old elements if they exist
            if (voltageEl) {
                voltageEl.textContent = voltage.toFixed(2);
                voltageEl.style.color = voltageColor;
            }
            if (fill) {
                fill.style.width = percent + '%';
                fill.className = `battery-fill ${fillClass}`;
            }
            if (statusEl) {
                statusEl.textContent = `${statusText} (${Math.round(percent)}%)`;
            }
            
            // Update new slim elements
            if (voltageSlimEl) {
                voltageSlimEl.textContent = voltageText;
                voltageSlimEl.style.color = voltageColor;
            }
            if (fillSlim) {
                fillSlim.style.width = percent + '%';
                fillSlim.className = `battery-fill-slim ${fillClass}`;
            }
            if (statusSlimEl) {
                statusSlimEl.textContent = statusText;
            }
        } else {
            // No data state
            if (voltageEl) {
                voltageEl.textContent = '--';
                voltageEl.style.color = 'rgba(255, 255, 255, 0.5)';
            }
            if (fill) {
                fill.style.width = '0%';
                fill.className = 'battery-fill battery-good';
            }
            if (statusEl) {
                statusEl.textContent = 'NO DATA';
            }
            
            // Update slim elements
            if (voltageSlimEl) {
                voltageSlimEl.textContent = '-- V';
                voltageSlimEl.style.color = 'rgba(255, 255, 255, 0.5)';
            }
            if (fillSlim) {
                fillSlim.style.width = '0%';
                fillSlim.className = 'battery-fill-slim battery-good';
            }
            if (statusSlimEl) {
                statusSlimEl.textContent = 'NO DATA';
            }
        }
    }

    updatePowerStatus(powerStatus) {
        const iconEl = document.getElementById('powerIcon');
        const statusEl = document.getElementById('powerStatus');

        // Guard against missing elements
        if (!iconEl || !statusEl) {
            return;
        }

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
        
        // Guard against missing element
        if (!updateEl) {
            return;
        }
        
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
