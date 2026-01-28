/**
 * ros_maps.js
 * - Handles Map Visualization with ROS2DJS.
 */

document.addEventListener("DOMContentLoaded", () => {
    const mapElements = document.getElementsByName('map-canvas');
    if (mapElements.length === 0) return;

    mapElements.forEach((container) => {
        initSingleMap(container);
    });
});

// --- GLOBAL ZOOM FUNCTION (Called by Python Buttons) ---
window.zoomMap = function (topicName, zoomFactor) {
    const container = document.getElementById(topicName);
    if (!container || !container.mapViewer) return;

    const viewer = container.mapViewer;
    applyZoom(viewer, zoomFactor, null); // Null center means zoom to center of view
};


function initSingleMap(container) {
    const topicName = container.id;
    if (!topicName) return;

    // --- 1. Setup Viewer ---
    // Start with a default size; it will resize when map arrives
    const viewer = new ROS2D.Viewer({
        divID: topicName,
        width: 1, height: 1, // Will be resized immediately
        background: '#7f7f7f'
    });

    // IMPORTANT: Attach viewer to the container so we can resize it later
    container.mapViewer = viewer;

    // Ensure Canvas is block to remove bottom font-baseline gap
    const canvas = viewer.scene.canvas;
    canvas.style.display = 'block';

    // Disable image smoothing for crisp grid lines (looks better for occupancy grids)
    const ctx = canvas.getContext("2d");
    if (ctx) {
        ctx.imageSmoothingEnabled = false;
    }

    // --- 2. Setup Mock ROS ---
    const mockRos = new EventEmitter2();
    mockRos.callOnConnection = () => { };
    mockRos.send = () => { };
    mockRos.connect = () => { };
    mockRos.close = () => { };
    mockRos.isConnected = true;

    // --- 3. Setup Grid Client ---
    const gridClient = new ROS2D.OccupancyGridClient({
        ros: mockRos,
        rootObject: viewer.scene,
        topic: topicName,
        continuous: false
    });

    // Attach gridClient to container so resizeMap can access bounds later
    container.mapGridClient = gridClient;

    // --- 4. Handle Rendering ---
    gridClient.on('change', () => {
        // Debounce resize to ensure we don't spam it during load
        requestAnimationFrame(() => resizeMap(container));
    });

    // --- 5. Connect WebSocket ---
    const protocol = window.location.protocol === "https:" ? "wss:" : "ws:";
    const wsUrl = `${protocol}//${window.location.host}/ws_${topicName}`;
    const ws = new WebSocket(wsUrl);
    ws.binaryType = 'arraybuffer';

    ws.onmessage = (event) => {
        try {
            let msgData;
            if (event.data instanceof ArrayBuffer) {
                const decoder = new TextDecoder("utf-8");
                msgData = JSON.parse(decoder.decode(event.data));
            } else {
                msgData = JSON.parse(event.data);
            }

            let mapMessage = msgData.msg ? msgData.msg : msgData;

            if (mapMessage.data && typeof mapMessage.data === 'string') {
                const binaryString = atob(mapMessage.data);
                const len = binaryString.length;
                const bytes = new Int8Array(len);
                for (let i = 0; i < len; i++) {
                    bytes[i] = binaryString.charCodeAt(i);
                }
                mapMessage.data = bytes;
            }

            mockRos.emit(topicName, mapMessage);

        } catch (e) {
            console.error(`[Map ${topicName}] Error:`, e);
        }
    };

    // --- 6. Interactions ---
    setupInteractions(viewer, container);

    // --- 7. Resize Observer ---
    const resizeObserver = new ResizeObserver(() => {
        resizeMap(container);
    });
    resizeObserver.observe(container);
}

/**
 * Apply Zoom helper
 * @param {Object} center - {x, y} in screen coordinates (optional)
 */
function applyZoom(viewer, factor, center) {
    const scene = viewer.scene;

    const newScale = scene.scaleX * factor;

    if (center) {
        // Zoom towards mouse point
        const localX = (center.x - scene.x) / scene.scaleX;
        const localY = (center.y - scene.y) / scene.scaleY;

        scene.scaleX = newScale;
        scene.scaleY = newScale;

        scene.x = center.x - localX * newScale;
        scene.y = center.y - localY * newScale;
    } else {
        // Zoom towards center of screen
        const centerX = viewer.width / 2;
        const centerY = viewer.height / 2;

        const localX = (centerX - scene.x) / scene.scaleX;
        const localY = (centerY - scene.y) / scene.scaleY;

        scene.scaleX = newScale;
        scene.scaleY = newScale;

        scene.x = centerX - localX * newScale;
        scene.y = centerY - localY * newScale;
    }
}


function setupInteractions(viewer, container) {
    const canvas = viewer.scene.canvas;
    if (!canvas) return;

    let isDragging = false;
    let lastX, lastY;

    // --- PAN (Mouse Down) ---
    canvas.addEventListener('mousedown', (event) => {
        if (event.button === 0 || event.button === 1) { // Left or Middle click
            isDragging = true;
            lastX = event.clientX;
            lastY = event.clientY;
            canvas.style.cursor = 'grabbing';
            event.preventDefault(); // Prevent text selection
        }
    });

    window.addEventListener('mousemove', (event) => {
        if (!isDragging) return;
        const dpr = window.devicePixelRatio || 1;
        const dx = (event.clientX - lastX) * dpr;
        const dy = (event.clientY - lastY) * dpr;
        viewer.scene.x += dx;
        viewer.scene.y += dy;
        lastX = event.clientX;
        lastY = event.clientY;
    });

    window.addEventListener('mouseup', () => {
        if (isDragging) {
            isDragging = false;
            canvas.style.cursor = 'default';
        }
    });

    // --- ZOOM (Scroll Wheel) ---
    canvas.addEventListener('wheel', (event) => {
        // Check if Fullscreen
        const isFullscreen = container.closest('.fullscreen-overlay') !== null;

        if (!isFullscreen) {
            // NOT Fullscreen: Do NOT capture scroll. Let page scroll.
            return;
        }

        // IS Fullscreen: Capture scroll and Zoom.
        event.preventDefault();
        event.stopPropagation();

        const zoomFactor = event.deltaY < 0 ? 1.1 : 0.9;

        // Calculate mouse position relative to canvas
        const rect = canvas.getBoundingClientRect();
        const dpr = window.devicePixelRatio || 1;

        const mouseX = (event.clientX - rect.left) * dpr;
        const mouseY = (event.clientY - rect.top) * dpr;

        applyZoom(viewer, zoomFactor, { x: mouseX, y: mouseY });
    }, { passive: false }); // Passive false required to use preventDefault
}

/**
 * Helper to manually resize the ROS2D Viewer canvas
 */
function resizeViewer(viewer, logicalWidth, logicalHeight) {
    const canvas = viewer.scene.canvas;
    const dpr = window.devicePixelRatio || 1;

    // 1. Set the "Physical" size (buffer size)
    const physicalWidth = Math.floor(logicalWidth * dpr);
    const physicalHeight = Math.floor(logicalHeight * dpr);

    canvas.width = physicalWidth;
    canvas.height = physicalHeight;
    viewer.width = physicalWidth;
    viewer.height = physicalHeight;

    // 2. Force the "Logical" size via CSS (how big it looks on screen)
    canvas.style.width = `${logicalWidth}px`;
    canvas.style.height = `${logicalHeight}px`;

    // 3. Ensure smoothing is disabled for 2D maps after resize
    const ctx = canvas.getContext("2d");
    if (ctx) ctx.imageSmoothingEnabled = false;
}

/**
 * Main Logic: Fits the Map to the Container
 */
function resizeMap(container) {
    if (!container || !container.mapViewer) return;

    const viewer = container.mapViewer;
    const gridClient = container.mapGridClient;
    const MAP_ROTATION = -90;

    // Wait slightly for CSS transitions
    setTimeout(() => {
        // 1. Get available CSS space
        const logicalWidth = container.clientWidth;
        const logicalHeight = container.clientHeight;

        if (logicalWidth === 0 || logicalHeight === 0) return;

        // 2. Resize Canvas
        resizeViewer(viewer, logicalWidth, logicalHeight);

        // 3. Fit Map Content
        if (gridClient && gridClient.currentGrid) {
            const grid = gridClient.currentGrid;

            // Update Grid Registration & Rotation
            grid.regX = grid.width / 2;
            grid.regY = grid.height / 2;
            grid.rotation = MAP_ROTATION;
            grid.x = grid.width / 2;
            grid.y = grid.height / 2;

            const bounds = grid.getTransformedBounds();
            if (!bounds) return;

            const isFullscreen = container.classList.contains('fullscreen-overlay');

            // We use viewer.width/height which are now High DPI (Physical) pixels.
            // This ensures the zoom level matches the physical pixel density.
            const canvasPhysWidth = viewer.width;
            const canvasPhysHeight = viewer.height;

            if (isFullscreen) {
                // A. Fullscreen: Contain map within the fixed container
                const zoom = Math.min(canvasPhysWidth / bounds.width, canvasPhysHeight / bounds.height);

                viewer.scene.scaleX = zoom;
                viewer.scene.scaleY = zoom;

                viewer.scene.x = (canvasPhysWidth - (bounds.width * zoom)) / 2 - (bounds.x * zoom);
                viewer.scene.y = (canvasPhysHeight - (bounds.height * zoom)) / 2 - (bounds.y * zoom);

            } else {
                // B. Standard Card: Fit container height to map aspect ratio
                const mapAspectRatio = bounds.width / bounds.height;
                const newLogicalHeight = logicalWidth / mapAspectRatio;

                // Apply height update to DOM
                container.style.height = `${newLogicalHeight}px`;

                // Re-run resizeViewer with new height
                resizeViewer(viewer, logicalWidth, newLogicalHeight);

                // Recalculate zoom with new physical dimensions
                const newPhysWidth = viewer.width;
                const zoom = newPhysWidth / bounds.width;

                viewer.scene.scaleX = zoom;
                viewer.scene.scaleY = zoom;

                viewer.scene.x = (newPhysWidth - (bounds.width * zoom)) / 2 - (bounds.x * zoom);
                // For non-fullscreen, we just centered height, so simple centering works
                viewer.scene.y = (viewer.height - (bounds.height * zoom)) / 2 - (bounds.y * zoom);
            }
        }
    }, 50);
}
