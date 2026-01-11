// init.js - Initialize all controllers

// Global controllers
let patrol = null;
let mapController = null;
let teleopController = null;
let pointcloudViewer = null;
let webTeleopController = null;
let powerController = null;
let dogFollower = null;
let shoeFollower = null;

document.addEventListener('DOMContentLoaded', () => {
    // Initialize dashboard (WebSocket connection)
    const dashboard = new Rover1Dashboard();

    // Initialize map controller
    mapController = new MapController();

    // Initialize patrol controller
    patrol = new PatrolController(dashboard);
    dashboard.setPatrolController(patrol);

    // Initialize teleop speed control
    teleopController = new TeleopController(dashboard);
    dashboard.setTeleopController(teleopController);

    // Initialize web teleop driving (touch/keyboard controls)
    webTeleopController = new WebTeleopController(dashboard);

    // Initialize power controls
    powerController = new PowerController(dashboard);

    // Initialize vision controllers (dog and shoe followers)
    dogFollower = new DogFollowerController(dashboard);
    shoeFollower = new ShoeFollowerController(dashboard);

    // Initialize point cloud viewer
    pointcloudViewer = new PointCloudViewer();

    // Make globally accessible for debugging
    window.dashboard = dashboard;
    window.patrol = patrol;
    window.mapController = mapController;
    window.teleopController = teleopController;
    window.webTeleopController = webTeleopController;
    window.powerController = powerController;
    window.dogFollower = dogFollower;
    window.shoeFollower = shoeFollower;
    window.pointcloudViewer = pointcloudViewer;

    console.log('Rover1 Dashboard initialized');
});
