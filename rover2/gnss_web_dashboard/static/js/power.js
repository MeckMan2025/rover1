// power.js - Shutdown and reboot controls
class PowerController {
    constructor(dashboard) {
        this.dashboard = dashboard;
        this.pendingAction = null;  // 'shutdown' or 'reboot'
    }

    requestShutdown() {
        this.pendingAction = 'shutdown';
        this.showConfirm('Shut down the rover? You will need physical access to turn it back on.');
    }

    requestReboot() {
        this.pendingAction = 'reboot';
        this.showConfirm('Reboot the rover? It will be back online in about 60 seconds.');
    }

    showConfirm(message) {
        document.getElementById('powerConfirmMsg').textContent = message;
        document.getElementById('powerConfirmDialog').classList.add('active');
    }

    hideConfirm() {
        document.getElementById('powerConfirmDialog').classList.remove('active');
        this.pendingAction = null;
    }

    cancelAction() {
        this.hideConfirm();
    }

    confirmAction() {
        if (!this.pendingAction) return;

        const action = this.pendingAction;
        this.hideConfirm();

        // Send command to server
        this.dashboard.sendCommand({ action: action });

        // Show feedback
        const msg = action === 'shutdown'
            ? 'Shutting down... Goodbye!'
            : 'Rebooting... See you in ~60 seconds!';

        document.getElementById('powerConfirmMsg').textContent = msg;
        document.getElementById('powerConfirmDialog').classList.add('active');
        document.querySelector('.power-confirm-btns').style.display = 'none';

        // For reboot, the page will eventually reconnect
        // For shutdown, user will need to close the page
    }
}


window.PowerController = PowerController;
