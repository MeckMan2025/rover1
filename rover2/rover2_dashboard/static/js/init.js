// init.js - Initialize rover2 controllers

// Global controllers
let mapController = null;
let teleopController = null;
let pointcloudViewer = null;
let webTeleopController = null;
let powerController = null;
let personFollower = null;

document.addEventListener('DOMContentLoaded', () => {
    // Initialize dashboard (WebSocket connection)
    const dashboard = new Rover1Dashboard();

    // Initialize map controller
    mapController = new MapController();

    // Initialize teleop speed control
    teleopController = new TeleopController(dashboard);
    dashboard.setTeleopController(teleopController);

    // Initialize web teleop driving (touch/keyboard controls)
    webTeleopController = new WebTeleopController(dashboard);

    // Initialize power controls
    powerController = new PowerController(dashboard);

    // Initialize person follower (rover2's main feature)
    personFollower = new PersonFollowerController(dashboard);

    // Initialize point cloud viewer
    pointcloudViewer = new PointCloudViewer();

    // Make globally accessible for debugging
    window.dashboard = dashboard;
    window.mapController = mapController;
    window.teleopController = teleopController;
    window.webTeleopController = webTeleopController;
    window.powerController = powerController;
    window.personFollower = personFollower;
    window.pointcloudViewer = pointcloudViewer;

    console.log('Rover2 Dashboard initialized');
});
