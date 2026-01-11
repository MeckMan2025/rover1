// map.js - Leaflet map integration
class MapController {
    constructor() {
        this.map = null;
        this.marker = null;
        this.accuracyCircle = null;
        this.followMode = true;
        this.hasPosition = false;
        this.lastLat = null;
        this.lastLon = null;
        this.pathLine = null;
        this.pathPoints = [];
        this.maxPathPoints = 500;

        this.initMap();
    }

    initMap() {
        // Default to a generic location until we get GPS fix
        const defaultLat = 39.8283;  // Center of US
        const defaultLon = -98.5795;
        const defaultZoom = 4;

        this.map = L.map('roverMap', {
            center: [defaultLat, defaultLon],
            zoom: defaultZoom,
            zoomControl: true
        });

        // Add OpenStreetMap tiles
        L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
            maxZoom: 19,
            attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a>'
        }).addTo(this.map);

        // Create custom rover icon
        this.roverIcon = L.divIcon({
            className: 'rover-marker',
            html: `<div style="
                width: 24px;
                height: 24px;
                background: #00ff88;
                border: 3px solid #fff;
                border-radius: 50%;
                box-shadow: 0 0 10px rgba(0,255,136,0.5);
            "></div>`,
            iconSize: [24, 24],
            iconAnchor: [12, 12]
        });

        // Create marker (hidden until we get position)
        this.marker = L.marker([defaultLat, defaultLon], {
            icon: this.roverIcon
        });

        // Create accuracy circle
        this.accuracyCircle = L.circle([defaultLat, defaultLon], {
            radius: 10,
            color: '#00ff88',
            fillColor: '#00ff88',
            fillOpacity: 0.1,
            weight: 2
        });

        // Create path line
        this.pathLine = L.polyline([], {
            color: '#0088ff',
            weight: 3,
            opacity: 0.7
        });
    }

    updatePosition(lat, lon, alt, rtkState) {
        // Update coordinate display
        document.getElementById('mapLat').textContent = lat !== null ? lat.toFixed(7) : '--';
        document.getElementById('mapLon').textContent = lon !== null ? lon.toFixed(7) : '--';
        document.getElementById('mapAlt').textContent = alt !== null ? alt.toFixed(1) + 'm' : '--';

        if (lat === null || lon === null) {
            return;
        }

        const position = [lat, lon];

        // First valid position - add marker to map and zoom in
        if (!this.hasPosition) {
            this.hasPosition = true;
            this.marker.addTo(this.map);
            this.accuracyCircle.addTo(this.map);
            this.pathLine.addTo(this.map);
            this.map.setView(position, 18);
        }

        // Update marker position
        this.marker.setLatLng(position);
        this.accuracyCircle.setLatLng(position);

        // Update marker color based on RTK state
        let color = '#ffc107';  // Default yellow
        if (rtkState === 'FIXED') {
            color = '#00ff88';
        } else if (rtkState === 'FLOAT') {
            color = '#0088ff';
        } else if (rtkState === 'NO_FIX') {
            color = '#ff4444';
        }

        this.marker.setIcon(L.divIcon({
            className: 'rover-marker',
            html: `<div style="
                width: 24px;
                height: 24px;
                background: ${color};
                border: 3px solid #fff;
                border-radius: 50%;
                box-shadow: 0 0 10px ${color}80;
            "></div>`,
            iconSize: [24, 24],
            iconAnchor: [12, 12]
        }));

        // Add to path if position changed significantly (> 0.5m)
        if (this.lastLat !== null && this.lastLon !== null) {
            const dist = this.haversineDistance(this.lastLat, this.lastLon, lat, lon);
            if (dist > 0.5) {
                this.pathPoints.push(position);
                if (this.pathPoints.length > this.maxPathPoints) {
                    this.pathPoints.shift();
                }
                this.pathLine.setLatLngs(this.pathPoints);
                this.lastLat = lat;
                this.lastLon = lon;
            }
        } else {
            this.lastLat = lat;
            this.lastLon = lon;
        }

        // Follow mode - keep rover centered
        if (this.followMode) {
            this.map.setView(position, this.map.getZoom(), { animate: true });
        }

        // Update popup
        this.marker.bindPopup(`
            <div class="rover-popup">
                <div class="status" style="color: ${color}">${rtkState || 'UNKNOWN'}</div>
                <div class="coords">${lat.toFixed(7)}, ${lon.toFixed(7)}</div>
            </div>
        `);
    }

    haversineDistance(lat1, lon1, lat2, lon2) {
        const R = 6371000; // Earth radius in meters
        const dLat = (lat2 - lat1) * Math.PI / 180;
        const dLon = (lon2 - lon1) * Math.PI / 180;
        const a = Math.sin(dLat/2) * Math.sin(dLat/2) +
                  Math.cos(lat1 * Math.PI / 180) * Math.cos(lat2 * Math.PI / 180) *
                  Math.sin(dLon/2) * Math.sin(dLon/2);
        const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1-a));
        return R * c;
    }

    centerOnRover() {
        if (this.hasPosition && this.lastLat !== null) {
            this.map.setView([this.lastLat, this.lastLon], 18, { animate: true });
        }
    }

    toggleFollow() {
        this.followMode = !this.followMode;
        document.getElementById('followBtn').textContent =
            this.followMode ? 'Follow: ON' : 'Follow: OFF';
        if (this.followMode) {
            this.centerOnRover();
        }
    }

    clearPath() {
        this.pathPoints = [];
        this.pathLine.setLatLngs([]);
    }
}


window.MapController = MapController;
