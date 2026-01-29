/**
 * ros_maps.js
 * - Handles Map Visualization with ROS2DJS.
 */

// State to track if we are in "Publish Point" mode
const mapInteractionState = {}; // { topicName: { isPublishing: boolean, btn: HTMLElement } }


document.addEventListener("DOMContentLoaded", () => {
    const mapElements = document.getElementsByName('map-canvas');
    if (mapElements.length === 0) return;

    mapElements.forEach((container) => {
        initSingleMap(container);
    });
});

// --- EXPORTED FUNCTIONS (Called by Python Buttons) ---
window.zoomMap = function (topicName, zoomFactor) {
    const container = document.getElementById(topicName);
    if (!container || !container.mapViewer) return;

    const viewer = container.mapViewer;
    applyZoom(viewer, zoomFactor, null); // Null center means zoom to center of view
};


window.togglePublishPoint = function (btn) {
    const mapElements = document.getElementsByName('map-canvas');
    // Check if no maps are found
    if (mapElements.length === 0) {
        if (typeof UIkit !== 'undefined') {
            UIkit.notification({
                message: "<span uk-icon='icon: warning'></span> No available input map elements found.",
                status: 'danger',
                pos: 'top-center',
                timeout: 5000
            });
        } else {
            console.warn("No map elements found.");
        }
        return;
    }

    // 1. Determine if we are turning ON or OFF based on button state
    const isActive = btn.classList.contains('active-brand-red');
    const isTurningOn = !isActive;

    // 2. Update Button UI
    if (isTurningOn) {
        btn.classList.remove('uk-button-default', 'bg-white/80', 'dark:bg-gray-800/80', 'backdrop-blur');
        btn.classList.add('active-brand-red');
    } else {
        btn.classList.remove('active-brand-red');
        btn.classList.add('uk-button-default', 'bg-white/80', 'dark:bg-gray-800/80', 'backdrop-blur');
    }

    // 3. Apply to ALL Maps
    mapElements.forEach(container => {
        const topicName = container.id;

        if (!mapInteractionState[topicName]) {
            mapInteractionState[topicName] = { isPublishing: false, btn: null };
        }
        const state = mapInteractionState[topicName];

        state.isPublishing = isTurningOn;
        // Save the current active button
        state.btn = btn;

        // Update Cursor
        container.style.cursor = isTurningOn ? 'crosshair' : 'default';

        // 4. Determine Settings if Turning On
        if (isTurningOn) {
            // CASE 1: Click came from the map's own settings button
            if (btn.id === `${topicName}-publish-btn`) {
                const settingsForm = document.getElementById(`${topicName}-settings-form`);

                let targetTopic = 'clicked_point';
                let msgType = 'PointStamped';

                if (state.settings) {
                    targetTopic = state.settings.topic;
                    msgType = state.settings.type;
                } else if (settingsForm) {
                    const tInput = settingsForm.querySelector('[name="clicked_point_topic"]');
                    let mInput = settingsForm.querySelector('input[name="clicked_point_type"]');
                    if (!mInput) mInput = settingsForm.querySelector('select[name="clicked_point_type"]');

                    if (tInput && tInput.value) targetTopic = tInput.value;
                    if (mInput && mInput.value) msgType = mInput.value;

                    // Cache these initial settings
                    state.settings = { topic: targetTopic, type: msgType };
                }

                // Save configuration for this button ID
                state[btn.id] = { topic: targetTopic, type: msgType };

            } else {
                // CASE 2: External/Generic Button
                // Read from data attributes, default to standard if missing
                const customTopic = btn.getAttribute('data-topic') || 'clicked_point';
                const customType = btn.getAttribute('data-type') || 'PointStamped';

                state[btn.id] = { topic: customTopic, type: customType };
            }
        }
    });
};

window.openMapSettings = function (topicName) {
    const modal = document.getElementById(`${topicName}-settings-modal`);
    if (!modal) return;

    // 1. Show Modal
    modal.style.display = 'grid';

    // 2. Restore Saved Values (if they exist)
    const state = mapInteractionState[topicName];
    if (state && state.settings) {
        const form = document.getElementById(`${topicName}-settings-form`);
        if (form) {
            // A. Restore Text Input
            const tInput = form.querySelector('input[name="clicked_point_topic"]');
            if (tInput) tInput.value = state.settings.topic;

            // B. Restore Dropdown (Complex Component)
            // We must find the HIDDEN NATIVE SELECT inside the custom component to update the UI
            // Selector: Find the uk-select with this name, then find the native select inside it
            const selectContainer = form.querySelector(`uk-select[name="clicked_point_type"]`);
            if (selectContainer) {
                const nativeSelect = selectContainer.querySelector('select');
                if (nativeSelect) {
                    nativeSelect.value = state.settings.type;
                    // Dispatch change event so the Custom UI updates its text
                    nativeSelect.dispatchEvent(new Event('change', { bubbles: true }));
                }
            } else {
                // Fallback: try finding any select with that name
                const simpleSelect = form.querySelector(`select[name="clicked_point_type"]`);
                if (simpleSelect) simpleSelect.value = state.settings.type;
            }
        }
    }
};

window.saveMapSettings = function (topicName) {
    const modal = document.getElementById(`${topicName}-settings-modal`);
    const form = document.getElementById(`${topicName}-settings-form`);

    // 1. Read & Save Values to Global State
    if (form) {
        // A. Get Topic (Standard Input)
        const tInput = form.querySelector('input[name="clicked_point_topic"]');

        // B. Get Type (Custom Component)
        // CRITICAL FIX: Explicitly select the 'input' tag to bypass the wrapper
        const mInput = form.querySelector('input[name="clicked_point_type"]');

        // Fallback checks if the specific input wasn't found (e.g., if structure changes)
        let typeValue = 'PointStamped';
        if (mInput) {
            typeValue = mInput.value;
        } else {
            // Try getting it from the select if the input is missing
            const sInput = form.querySelector('select[name="clicked_point_type"]');
            if (sInput) typeValue = sInput.value;
        }

        if (!mapInteractionState[topicName]) mapInteractionState[topicName] = {};

        mapInteractionState[topicName].settings = {
            topic: tInput ? tInput.value : 'clicked_point',
            type: typeValue
        };

        console.log(`[${topicName}] Settings Saved:`, mapInteractionState[topicName].settings);
    }

    // 2. Hide Modal
    if (modal) modal.style.display = 'none';
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
    container.mapWs = ws; // Store WS for sending data back
    container.topicName = topicName; // Store the map topic name

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

            // --- Save Map Metadata for Click Math ---
            if (mapMessage.info) {
                container.mapInfo = mapMessage.info;
            }
            // --- Save Map Header ---
            if (mapMessage.header) {
                container.mapHeader = mapMessage.header;
            }

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
        } catch (e) { console.error(e); }
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
 * Coordinate Transformation Logic
 * Screen (Pixels) -> Viewer (Pan/Zoom) -> Grid (Rotated) -> ROS (Meters)
 */
function transformScreenToRos(container, screenX, screenY) {
    const viewer = container.mapViewer;
    const gridClient = container.mapGridClient;
    const grid = gridClient.currentGrid;

    if (!grid) return null;

    // 1. Screen -> Stage (Account for Pan/Zoom)
    const dpr = window.devicePixelRatio || 1;
    const stageX = (screenX * dpr - viewer.scene.x) / viewer.scene.scaleX;
    const stageY = (screenY * dpr - viewer.scene.y) / viewer.scene.scaleY;

    // 2. Stage -> Grid (Account for Grid Position & Rotation)
    const dx = stageX - grid.x;
    const dy = stageY - grid.y;

    const rad = -grid.rotation * (Math.PI / 180.0); // Inverting the rotation
    const unrotatedX = dx * Math.cos(rad) - dy * Math.sin(rad);
    const unrotatedY = dx * Math.sin(rad) + dy * Math.cos(rad);

    const imageX = unrotatedX + grid.regX;
    const imageY = unrotatedY + grid.regY;

    // 3. Grid Pixels -> ROS Coordinates (Resolution & Origin)
    // --- KEY FIX: Use the stored info from container ---
    const info = container.mapInfo;

    if (!info) {
        console.warn("[Map] Metadata missing. Map not loaded yet?");
        return null;
    }

    const res = info.resolution;
    const originX = info.origin.position.x;
    const originY = info.origin.position.y;
    const mapHeight = info.height;

    // Calculate ROS coordinates (Assuming standard ROS bottom-left origin vs Canvas top-left)
    const rosX = (imageX * res) + originX;
    const rosY = ((mapHeight - imageY) * res) + originY;

    return { x: rosX, y: rosY, z: 0.0 };
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

/**
 * Helper method to publish a clicked point on a map canvas
 */
function publishPoint(container, targetTopic, rosPoint, msgType) {
    if (!container || !container.mapWs) {
        console.warn("Cannot publish point: Websocket or Container missing");
        return;
    }

    // Default Orientation
    const defaultOrientation = { ori_x: 0.0, ori_y: 0.0, ori_z: 0.0, ori_w: 1.0 };
    let messageData = {};

    // Safely get frame_id (Default to 'map' if header is missing)
    const frameId = (container.mapHeader && container.mapHeader.frame_id) ? container.mapHeader.frame_id : 'map';

    if (msgType === 'Point' || msgType === 'PointStamped') {
        messageData = rosPoint; // {x, y, z}
    } else if (msgType === 'Pose' || msgType === 'PoseStamped') {
        messageData = { ...rosPoint, ...defaultOrientation};
    }

    const payload = {
        topic_name: targetTopic,
        frame_id: frameId,
        topic_type: msgType,
        data: messageData
    };

    container.mapWs.send(JSON.stringify(payload));
    console.log(`Published [${msgType}] to [${targetTopic}]`, payload);
};


function setupInteractions(viewer, container) {
    const canvas = viewer.scene.canvas;
    if (!canvas) return;

    let isDragging = false;
    let lastX, lastY;

    // --- MOUSE DOWN (Handle Clicked Point or Pan) ---
    canvas.addEventListener('mousedown', (event) => {
        let topicName = container.topicName;
        const state = mapInteractionState[topicName];
        // --- PUBLISH POINT LOGIC ---
        if (state && state.isPublishing && event.button === 0) {
            const rect = canvas.getBoundingClientRect();
            const mouseX = event.clientX - rect.left;
            const mouseY = event.clientY - rect.top;

            // 1. Get raw point
            const rosPoint = transformScreenToRos(container, mouseX, mouseY);

            if (rosPoint) {
                // Determine which settings to use based on the active button
                let targetTopic = 'clicked_point';
                let msgType = 'PointStamped';

                if (state.btn && state[state.btn.id]) {
                    targetTopic = state[state.btn.id].topic;
                    msgType = state[state.btn.id].type;
                }

                publishPoint(container, targetTopic, rosPoint, msgType);
            }
            // Toggle OFF (Global function, passes the active button)
            window.togglePublishPoint(state.btn);
            return;
        }

        // --- PAN LOGIC ---
        if (event.button === 0 || event.button === 1) {
            isDragging = true;
            lastX = event.clientX;
            lastY = event.clientY;
            canvas.style.cursor = 'grabbing';
            event.preventDefault();
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
