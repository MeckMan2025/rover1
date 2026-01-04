// pointcloud.js - Three.js 3D point cloud visualization
class PointCloudViewer {
    constructor() {
        this.scene = null;
        this.camera = null;
        this.renderer = null;
        this.controls = null;
        this.points = null;
        this.axesHelper = null;
        this.showAxes = true;
        this.animationId = null;
        this.isInitialized = false;
        this.pointCount = 0;
    }

    init() {
        if (this.isInitialized) return;

        const container = document.getElementById('pointcloudContainer');
        const width = container.clientWidth;
        const height = container.clientHeight;

        // Scene
        this.scene = new THREE.Scene();
        this.scene.background = new THREE.Color(0x111111);

        // Camera
        this.camera = new THREE.PerspectiveCamera(60, width / height, 0.1, 1000);
        this.camera.position.set(5, 5, 5);
        this.camera.lookAt(0, 0, 0);

        // Renderer
        this.renderer = new THREE.WebGLRenderer({ antialias: true });
        this.renderer.setSize(width, height);
        this.renderer.setPixelRatio(window.devicePixelRatio);
        container.appendChild(this.renderer.domElement);

        // OrbitControls (inline implementation since we can't load external module)
        this.setupControls();

        // Axes helper
        this.axesHelper = new THREE.AxesHelper(2);
        this.scene.add(this.axesHelper);

        // Grid helper
        const gridHelper = new THREE.GridHelper(10, 20, 0x444444, 0x222222);
        this.scene.add(gridHelper);

        // Ambient light
        const ambientLight = new THREE.AmbientLight(0xffffff, 0.5);
        this.scene.add(ambientLight);

        // Handle resize
        window.addEventListener('resize', () => this.onResize());

        this.isInitialized = true;
        this.animate();
    }

    setupControls() {
        // Simple orbit controls using mouse events
        let isMouseDown = false;
        let prevMouseX = 0, prevMouseY = 0;
        let theta = Math.PI / 4, phi = Math.PI / 4;
        let radius = 10;
        const target = new THREE.Vector3(0, 0, 0);

        const container = document.getElementById('pointcloudContainer');

        const updateCamera = () => {
            this.camera.position.x = target.x + radius * Math.sin(phi) * Math.cos(theta);
            this.camera.position.y = target.y + radius * Math.cos(phi);
            this.camera.position.z = target.z + radius * Math.sin(phi) * Math.sin(theta);
            this.camera.lookAt(target);
        };

        container.addEventListener('mousedown', (e) => {
            isMouseDown = true;
            prevMouseX = e.clientX;
            prevMouseY = e.clientY;
        });

        container.addEventListener('mousemove', (e) => {
            if (!isMouseDown) return;
            const deltaX = e.clientX - prevMouseX;
            const deltaY = e.clientY - prevMouseY;
            theta -= deltaX * 0.01;
            phi = Math.max(0.1, Math.min(Math.PI - 0.1, phi + deltaY * 0.01));
            updateCamera();
            prevMouseX = e.clientX;
            prevMouseY = e.clientY;
        });

        container.addEventListener('mouseup', () => { isMouseDown = false; });
        container.addEventListener('mouseleave', () => { isMouseDown = false; });

        container.addEventListener('wheel', (e) => {
            e.preventDefault();
            radius = Math.max(1, Math.min(50, radius + e.deltaY * 0.01));
            updateCamera();
        });

        // Store for reset
        this.resetCameraFn = () => {
            theta = Math.PI / 4;
            phi = Math.PI / 4;
            radius = 10;
            target.set(0, 0, 0);
            updateCamera();
        };

        updateCamera();
    }

    onResize() {
        if (!this.isInitialized) return;
        const container = document.getElementById('pointcloudContainer');
        const width = container.clientWidth;
        const height = container.clientHeight;
        this.camera.aspect = width / height;
        this.camera.updateProjectionMatrix();
        this.renderer.setSize(width, height);
    }

    animate() {
        this.animationId = requestAnimationFrame(() => this.animate());
        this.renderer.render(this.scene, this.camera);
    }

    parsePLY(plyText) {
        const lines = plyText.split('\n');
        let vertexCount = 0;
        let headerEnd = 0;

        // Parse header
        for (let i = 0; i < lines.length; i++) {
            const line = lines[i].trim();
            if (line.startsWith('element vertex')) {
                vertexCount = parseInt(line.split(' ')[2]);
            }
            if (line === 'end_header') {
                headerEnd = i + 1;
                break;
            }
        }

        // Parse vertices
        const positions = new Float32Array(vertexCount * 3);
        const colors = new Float32Array(vertexCount * 3);

        for (let i = 0; i < vertexCount; i++) {
            const line = lines[headerEnd + i];
            if (!line) continue;
            const parts = line.trim().split(/\s+/);
            if (parts.length < 6) continue;

            positions[i * 3] = parseFloat(parts[0]);
            positions[i * 3 + 1] = parseFloat(parts[2]); // Swap Y and Z for Three.js
            positions[i * 3 + 2] = -parseFloat(parts[1]);

            colors[i * 3] = parseInt(parts[3]) / 255;
            colors[i * 3 + 1] = parseInt(parts[4]) / 255;
            colors[i * 3 + 2] = parseInt(parts[5]) / 255;
        }

        return { positions, colors, count: vertexCount };
    }

    loadPointCloud(base64Data) {
        // Decode base64
        const plyText = atob(base64Data);
        const { positions, colors, count } = this.parsePLY(plyText);

        this.pointCount = count;

        // Remove old points
        if (this.points) {
            this.scene.remove(this.points);
            this.points.geometry.dispose();
            this.points.material.dispose();
        }

        // Create geometry
        const geometry = new THREE.BufferGeometry();
        geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
        geometry.setAttribute('color', new THREE.BufferAttribute(colors, 3));

        // Center the point cloud
        geometry.computeBoundingBox();
        const center = new THREE.Vector3();
        geometry.boundingBox.getCenter(center);
        geometry.translate(-center.x, -center.y, -center.z);

        // Create material

window.PointCloudViewer = PointCloudViewer;
