// teleop.js - Speed control and web driving
class TeleopController {
    constructor(dashboard) {
        this.dashboard = dashboard;
        this.currentSpeed = 100;  // Percentage (10-100)
        this.debounceTimer = null;
        this.userAdjusting = false;  // Cooldown flag to prevent sync fighting
        this.syncCooldownTimer = null;
    }

    updateSpeed(value) {
        this.currentSpeed = parseInt(value);

        // Block server sync while user is adjusting
        this.userAdjusting = true;
        clearTimeout(this.syncCooldownTimer);
        this.syncCooldownTimer = setTimeout(() => {
            this.userAdjusting = false;
        }, 1000);  // 1 second cooldown after last adjustment

        // Update display immediately
        document.getElementById('teleopSpeedValue').textContent = this.currentSpeed + '%';

        // Update slider thumb color based on speed
        const slider = document.getElementById('teleopSpeedSlider');
        let color;
        if (this.currentSpeed >= 70) {
            color = '#00ff88';
        } else if (this.currentSpeed >= 40) {
            color = '#ffc107';
        } else {
            color = '#ff4444';
        }
        document.getElementById('teleopSpeedValue').style.color = color;

        // Debounce the WebSocket command to avoid flooding
        clearTimeout(this.debounceTimer);
        this.debounceTimer = setTimeout(() => {
            this.sendSpeedCommand();
        }, 100);
    }

    sendSpeedCommand() {
        const speedPercent = this.currentSpeed / 100;  // Convert to 0.1-1.0 range
        this.dashboard.sendCommand({
            action: 'set_teleop_speed',
            speed_percent: speedPercent
        });
    }

    syncFromServer(speedPercent) {
        // Skip sync if user is actively adjusting (prevents fighting)
        if (this.userAdjusting) return;

        // Sync slider when server broadcasts current speed
        const percent = Math.round(speedPercent * 100);
        if (percent !== this.currentSpeed) {
            this.currentSpeed = percent;
            document.getElementById('teleopSpeedSlider').value = percent;
            document.getElementById('teleopSpeedValue').textContent = percent + '%';
        }
    }
}

class WebTeleopController {
    constructor(dashboard) {
        this.dashboard = dashboard;
        this.activeActions = new Set();
        this.sendInterval = null;
        this.SEND_RATE_MS = 50;  // Send commands at 20Hz

        // Track touches by identifier -> action mapping for proper multi-touch
        this.activeTouches = new Map();

        // Key mappings: W=fwd, S=back, A=rot-left, D=rot-right, Q=strafe-left, E=strafe-right
        this.keyMap = {
            'KeyW': 'forward',
            'KeyS': 'backward',
            'KeyA': 'rotate-left',
            'KeyD': 'rotate-right',
            'KeyQ': 'strafe-left',
            'KeyE': 'strafe-right'
        };

        // Button ID to action mapping
        this.buttonMap = {
            'btnForward': 'forward',
            'btnBackward': 'backward',
            'btnRotateLeft': 'rotate-left',
            'btnRotateRight': 'rotate-right',
            'btnStrafeLeft': 'strafe-left',
            'btnStrafeRight': 'strafe-right'
        };

        // Action to velocity mapping
        this.actionVelocities = {
            'forward':      { linear_x:  1.0, linear_y:  0.0, angular_z:  0.0 },
            'backward':     { linear_x: -1.0, linear_y:  0.0, angular_z:  0.0 },
            'rotate-left':  { linear_x:  0.0, linear_y:  0.0, angular_z:  1.0 },
            'rotate-right': { linear_x:  0.0, linear_y:  0.0, angular_z: -1.0 },
            'strafe-left':  { linear_x:  0.0, linear_y:  1.0, angular_z:  0.0 },
            'strafe-right': { linear_x:  0.0, linear_y: -1.0, angular_z:  0.0 }
        };

        this.setupKeyboardListeners();
        this.setupMultiTouchListeners();
        this.setupMouseListeners();
    }

    setupKeyboardListeners() {
        document.addEventListener('keydown', (e) => {
            if (e.target.tagName === 'INPUT' || e.target.tagName === 'TEXTAREA') return;
            const action = this.keyMap[e.code];
            if (action && !this.activeActions.has(action)) {
                e.preventDefault();
                this.startAction(action);
                this.highlightButton(action, true);
            }
        });

        document.addEventListener('keyup', (e) => {
            const action = this.keyMap[e.code];
            if (action) {
                e.preventDefault();
                this.stopAction(action);
                this.highlightButton(action, false);
            }
        });

        window.addEventListener('blur', () => this.stopAll());
    }

    // PROPER MULTI-TOUCH: Container-level touch handling with touch identifier tracking
    setupMultiTouchListeners() {
        const container = document.getElementById('teleopContainer');
        if (!container) return;

        // Prevent all default touch behaviors on the container
        container.addEventListener('touchstart', (e) => this.handleTouchStart(e), { passive: false });
        container.addEventListener('touchmove', (e) => this.handleTouchMove(e), { passive: false });
        container.addEventListener('touchend', (e) => this.handleTouchEnd(e), { passive: false });
        container.addEventListener('touchcancel', (e) => this.handleTouchEnd(e), { passive: false });
    }

    // Find which button (if any) a touch point is over
    getButtonAtPoint(x, y) {
        const element = document.elementFromPoint(x, y);
        if (!element) return null;

        // Walk up to find the button
        let btn = element.closest('.teleop-btn');
        if (btn && btn.dataset.action) {
            return { button: btn, action: btn.dataset.action };
        }
        return null;
    }

    handleTouchStart(e) {
        e.preventDefault();

        for (const touch of e.changedTouches) {
            const result = this.getButtonAtPoint(touch.clientX, touch.clientY);
            if (result) {
                // Track this touch -> action
                this.activeTouches.set(touch.identifier, result.action);
                this.startAction(result.action);
                result.button.classList.add('active');
            }
        }
    }

    handleTouchMove(e) {
        e.preventDefault();

        // Check if any touch has moved to a different button
        for (const touch of e.changedTouches) {
            const oldAction = this.activeTouches.get(touch.identifier);
            const result = this.getButtonAtPoint(touch.clientX, touch.clientY);
            const newAction = result ? result.action : null;

            // Touch moved off its button
            if (oldAction && oldAction !== newAction) {
                this.stopAction(oldAction);
                this.highlightButton(oldAction, false);
                this.activeTouches.delete(touch.identifier);
            }

            // Touch moved onto a new button
            if (newAction && newAction !== oldAction) {
                this.activeTouches.set(touch.identifier, newAction);
                this.startAction(newAction);
                result.button.classList.add('active');
            }
        }
    }

    handleTouchEnd(e) {
        e.preventDefault();

        for (const touch of e.changedTouches) {
            const action = this.activeTouches.get(touch.identifier);
            if (action) {
                this.activeTouches.delete(touch.identifier);

                // Only stop if no other touch is still on this action
                let stillTouched = false;
                for (const a of this.activeTouches.values()) {
                    if (a === action) {
                        stillTouched = true;
                        break;
                    }
                }

                if (!stillTouched) {
                    this.stopAction(action);
                    this.highlightButton(action, false);
                }
            }
        }

        // Safety: if no touches left, make sure we stop
        if (this.activeTouches.size === 0 && this.activeActions.size > 0) {
            this.stopAll();
        }
    }

    // Mouse support for desktop
    setupMouseListeners() {
        const buttons = document.querySelectorAll('.teleop-btn');

        buttons.forEach(btn => {
            const action = btn.dataset.action;
            if (!action) return;

            btn.addEventListener('mousedown', (e) => {
                e.preventDefault();
                this.startAction(action);
                btn.classList.add('active');

                // Track mouse button state
                const handleMouseUp = () => {
                    this.stopAction(action);
                    btn.classList.remove('active');
                    document.removeEventListener('mouseup', handleMouseUp);
                };
                document.addEventListener('mouseup', handleMouseUp);
            });

            btn.addEventListener('mouseleave', () => {
                if (this.activeActions.has(action)) {
                    this.stopAction(action);
                    btn.classList.remove('active');
                }
            });
        });
    }

    highlightButton(action, active) {
        const idMap = {
            'forward': 'btnForward',
            'backward': 'btnBackward',
            'rotate-left': 'btnRotateLeft',
            'rotate-right': 'btnRotateRight',
            'strafe-left': 'btnStrafeLeft',
            'strafe-right': 'btnStrafeRight'
        };
        const btn = document.getElementById(idMap[action]);
        if (btn) {
            btn.classList.toggle('active', active);
        }
    }

    startAction(action) {
        if (!this.activeActions.has(action)) {
            this.activeActions.add(action);
            this.updateStatus();
            this.startSending();
        }
    }

    stopAction(action) {
        if (this.activeActions.has(action)) {
            this.activeActions.delete(action);
            this.updateStatus();
            if (this.activeActions.size === 0) {
                this.stopSending();
                this.sendCommand({ linear_x: 0, linear_y: 0, angular_z: 0 });
            }
        }
    }

    stopAll() {
        this.activeActions.clear();
        this.activeTouches.clear();
        document.querySelectorAll('.teleop-btn').forEach(btn => btn.classList.remove('active'));
        this.updateStatus();
        this.stopSending();
        this.sendCommand({ linear_x: 0, linear_y: 0, angular_z: 0 });
    }

    startSending() {
        if (this.sendInterval) return;
        this.sendInterval = setInterval(() => this.sendCurrentVelocity(), this.SEND_RATE_MS);
        this.sendCurrentVelocity();
    }

    stopSending() {
        if (this.sendInterval) {
            clearInterval(this.sendInterval);
            this.sendInterval = null;
        }
    }

    sendCurrentVelocity() {
        let linear_x = 0, linear_y = 0, angular_z = 0;

        this.activeActions.forEach(action => {
            const vel = this.actionVelocities[action];
            if (vel) {
                linear_x += vel.linear_x;
                linear_y += vel.linear_y;
                angular_z += vel.angular_z;
            }
        });

        linear_x = Math.max(-1, Math.min(1, linear_x));
        linear_y = Math.max(-1, Math.min(1, linear_y));
        angular_z = Math.max(-1, Math.min(1, angular_z));

        this.sendCommand({ linear_x, linear_y, angular_z });
    }

    sendCommand(velocity) {
        this.dashboard.sendCommand({
            action: 'teleop',
            linear_x: velocity.linear_x,
            linear_y: velocity.linear_y,
            angular_z: velocity.angular_z
        });
    }

    updateStatus() {
        const statusEl = document.getElementById('teleopStatus');
        if (this.activeActions.size > 0) {
            const actions = Array.from(this.activeActions).join(' + ');
            statusEl.innerHTML = `<span class="teleop-status-active">${actions.toUpperCase()}</span>`;
        } else {
            statusEl.innerHTML = `<span class="teleop-status-inactive">Ready - use both thumbs for agile control</span>`;
        }
    }
}


window.TeleopController = TeleopController;
window.WebTeleopController = WebTeleopController;
