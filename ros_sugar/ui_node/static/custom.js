/**
 * custom.js
 * Handles general UI interactions, Drag & Drop, HTMX configs, and logging.
 */

// Helper to persist forms
function persistForm(form) {
    if (typeof FormPersistence !== "undefined") {
        FormPersistence.persist(form);
    }
}

document.addEventListener("DOMContentLoaded", () => {
    // 1. Logging Auto-scroll
    const log_parent = document.getElementById("logging-card-parent");
    if (log_parent) {
        const observer = new MutationObserver(() => {
            const log = document.getElementById("outputs-log");
            if (log) log.scrollTop = log.scrollHeight;
        });
        observer.observe(log_parent, { childList: true, subtree: true });
    }

    // 2. Actions WebSocket (Frontend <-> Backend general comms)
    const actions = document.getElementById("actions-frontend");
    if (actions) {
        const wsProtocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
        const ws_actions = new WebSocket(`${wsProtocol}//${window.location.host}/ws_actions`);

        ws_actions.onopen = () => {
            console.log("Actions Websocket connection established");
        };
    }

    // 3. Initialize Draggables
    makeDraggable();
});

// Fix parsing boolean values from UI Switch elements for HTMX
document.addEventListener("htmx:configRequest", function (event) {
    const form = event.detail.elt.closest("form");
    if (!form) return;

    // Find all checkboxes or FastHTML Switch elements
    form.querySelectorAll("input[type='checkbox'], .Switch input").forEach(el => {
        const name = el.name;
        if (!name) return;
        // Always include the key, even if unchecked
        event.detail.parameters[name] = el.checked ? '1' : '0';
    });
});

// --- Draggable Logic ---

let highestZ = 1; // Global tracker for z-index

function makeDraggable(className = "draggable") {
    const elements = Array.from(document.getElementsByClassName(className));

    elements.forEach(el => {
        let isDragging = false;
        let offsetX = 0, offsetY = 0;

        const startDrag = (clientX, clientY) => {
            isDragging = true;
            highestZ++;
            el.style.zIndex = highestZ;

            const rect = el.getBoundingClientRect();

            // Standardizing size/position before moving
            const computedStyle = window.getComputedStyle(el);
            el.style.width = computedStyle.width;
            el.style.height = computedStyle.height;
            el.style.position = "absolute";
            el.style.left = `${rect.left}px`;
            el.style.top = `${rect.top}px`;

            offsetX = clientX - rect.left;
            offsetY = clientY - rect.top;
        };

        const handleDragStart = (e, clientX, clientY) => {
            // Is the user clicking the specific Handle?
            const isHandle = e.target.closest(".drag-handle");

            // Is the user clicking the Container background/padding directly?
            const isContainer = e.target === el;

            // If it's neither, we return (allowing text selection on children)
            if (!isHandle && !isContainer) return;

            //Still block inputs/buttons if they happen to be inside the Handle
            if (["INPUT", "BUTTON", "SELECT", "TEXTAREA"].includes(e.target.tagName)) return;

            e.preventDefault(); // Stop text selection while dragging
            startDrag(clientX, clientY);
        };

        el.addEventListener("mousedown", (e) => {
            handleDragStart(e, e.clientX, e.clientY);
        });

        document.addEventListener("mousemove", (e) => {
            if (!isDragging) return;
            el.style.left = `${e.clientX - offsetX}px`;
            el.style.top = `${e.clientY - offsetY}px`;
        });

        document.addEventListener("mouseup", () => {
            if (isDragging) {
                isDragging = false;
            }
        });

        // Touch support
        el.addEventListener("touchstart", (e) => {
            const touch = e.touches[0];
            handleDragStart(e, touch.clientX, touch.clientY);
        });

        document.addEventListener("touchmove", (e) => {
            if (!isDragging) return;
            if (e.cancelable) e.preventDefault();
            const touch = e.touches[0];
            el.style.left = `${touch.clientX - offsetX}px`;
            el.style.top = `${touch.clientY - offsetY}px`;
        }, { passive: false });

        document.addEventListener("touchend", () => {
            isDragging = false;
        });
    });
}
