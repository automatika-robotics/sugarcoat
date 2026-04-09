/**
 * system_graph.js
 * Lays out a mixed graph of components, events, and recipe-level actions.
 * Components = rectangles, events = diamonds, recipe actions = ovals.
 * All nodes absolutely positioned; edges are SVG bezier curves.
 */

const NODE_H_GAP = 60;
const NODE_V_GAP = 130;
const PADDING_X = 40;
const PADDING_Y = 60;

const EDGE_PALETTE = [
    "#E83F3F", "#447AE5", "#2ECC71", "#9B59B6", "#F1C40F",
    "#E67E22", "#1ABC9C", "#FF69B4", "#00CED1", "#FF6347",
];

// Entry corners for event diamond inputs (bottom reserved for outputs)
const ENTRY_CORNERS = ["top", "left", "right"];

// Persistent graph state for edge redrawing during drag
let _graphState = null;
// When true, suppress automatic re-layout (user has manually positioned nodes)
let _userDragged = false;


// ─── SVG Helpers ──────────────────────────────────────────────

/** Create an SVG element with given tag and attributes. */
function svgEl(tag, attrs) {
    const el = document.createElementNS("http://www.w3.org/2000/svg", tag);
    for (const [k, v] of Object.entries(attrs)) {
        if (k === "style" && typeof v === "object") {
            for (const [sk, sv] of Object.entries(v)) el.style[sk] = sv;
        } else {
            el.setAttribute(k, v);
        }
    }
    return el;
}

/** Get the anchor point on an element's bounding box for a given side. */
function getAnchor(rect, cRect, side) {
    const cx = rect.left + rect.width / 2 - cRect.left;
    const cy = rect.top + rect.height / 2 - cRect.top;
    switch (side) {
        case "top":    return { x: cx, y: rect.top - cRect.top };
        case "bottom": return { x: cx, y: rect.bottom - cRect.top };
        case "left":   return { x: rect.left - cRect.left, y: cy };
        case "right":  return { x: rect.right - cRect.left, y: cy };
        default:       return { x: cx, y: rect.top - cRect.top };
    }
}

/** Get arrowhead points for a given anchor point and direction. */
function arrowPoints(x, y, direction, size) {
    const s = size || 6;
    switch (direction) {
        case "down":  return `${x},${y} ${x - s},${y - s * 1.5} ${x + s},${y - s * 1.5}`;
        case "right": return `${x},${y} ${x - s * 1.5},${y - s} ${x - s * 1.5},${y + s}`;
        case "left":  return `${x},${y} ${x + s * 1.5},${y - s} ${x + s * 1.5},${y + s}`;
        default:      return `${x},${y} ${x - s},${y - s * 1.5} ${x + s},${y - s * 1.5}`;
    }
}

/** Direction the arrow should point when entering a given side. */
function arrowDirection(side) {
    switch (side) {
        case "left":  return "right";
        case "right": return "left";
        default:      return "down";
    }
}


// ─── Edge Drawing ─────────────────────────────────────────────

/** Draw a bezier edge between two anchor points with an arrowhead and optional label. */
function drawEdge(svg, src, dst, entrySide, label, color, dashed, fromId, toId) {
    const curvature = Math.max(Math.abs(dst.y - src.y) * 0.4, 40);

    let d;
    if (entrySide === "left") {
        d = `M ${src.x} ${src.y} C ${src.x} ${src.y + curvature}, ${Math.min(src.x, dst.x) - curvature} ${dst.y}, ${dst.x} ${dst.y}`;
    } else if (entrySide === "right") {
        d = `M ${src.x} ${src.y} C ${src.x} ${src.y + curvature}, ${Math.max(src.x, dst.x) + curvature} ${dst.y}, ${dst.x} ${dst.y}`;
    } else {
        d = `M ${src.x} ${src.y} C ${src.x} ${src.y + curvature}, ${dst.x} ${dst.y - curvature}, ${dst.x} ${dst.y}`;
    }

    const dataAttrs = { "data-from": fromId, "data-to": toId };

    const path = svgEl("path", {
        d, class: "topic-line", ...dataAttrs,
        style: { stroke: color, strokeDasharray: dashed ? "6 4" : "" },
    });
    svg.appendChild(path);

    const arrow = svgEl("polygon", {
        points: arrowPoints(dst.x, dst.y, arrowDirection(entrySide)),
        class: "topic-arrow", ...dataAttrs,
        style: { fill: color },
    });
    svg.appendChild(arrow);

    if (label) {
        const shortLabel = label.startsWith("/") ? label.slice(1) : label;
        const text = svgEl("text", {
            x: (src.x + dst.x) / 2, y: (src.y + dst.y) / 2 - 4,
            class: "topic-label", "text-anchor": "middle", ...dataAttrs,
            style: { fill: color },
        });
        text.textContent = shortLabel;
        svg.appendChild(text);
    }
}

/** Draw an external stub (line + arrow + label) arriving at or leaving from a point. */
function drawStub(svg, x, y, direction, label, color, nodeId) {
    const stubLen = 30;
    const as = 4;
    let x1, y1, labelX, labelY, textAnchor, arrowPts;

    if (direction === "from-top") {
        x1 = x; y1 = y - stubLen;
        labelX = x; labelY = y1 - 5; textAnchor = "middle";
        arrowPts = arrowPoints(x, y, "down", as);
    } else if (direction === "from-bottom") {
        x1 = x; y1 = y + stubLen;
        labelX = x; labelY = y1 + 12; textAnchor = "middle";
        arrowPts = arrowPoints(x, y1, "down", as);
    } else if (direction === "from-left") {
        x1 = x - stubLen; y1 = y;
        labelX = x1 - 4; labelY = y + 3; textAnchor = "end";
        arrowPts = arrowPoints(x, y, "right", as);
    } else { // from-right
        x1 = x + stubLen; y1 = y;
        labelX = x1 + 4; labelY = y + 3; textAnchor = "start";
        arrowPts = arrowPoints(x, y, "left", as);
    }

    const dataAttr = { "data-node": nodeId || "" };
    const shortName = label.startsWith("/") ? label.slice(1) : label;

    svg.appendChild(svgEl("line", {
        x1, y1, x2: x, y2: y, class: "topic-stub", ...dataAttr, style: { stroke: color },
    }));
    svg.appendChild(svgEl("polygon", {
        points: arrowPts, class: "topic-stub-arrow", ...dataAttr, style: { fill: color },
    }));
    const text = svgEl("text", {
        x: labelX, y: labelY, class: "topic-stub-label", "text-anchor": textAnchor,
        ...dataAttr, style: { fill: color },
    });
    text.textContent = shortName;
    svg.appendChild(text);
}


// ─── Topic Utilities ──────────────────────────────────────────

function normalizeTopic(t) {
    return t.startsWith("/") ? t.slice(1) : t;
}


// ─── Main ─────────────────────────────────────────────────────

function drawTopicConnections() {
    // If user has manually dragged nodes, only redraw edges — don't re-layout
    if (_userDragged && _graphState) {
        redrawAllEdges();
        return;
    }

    const container = document.getElementById("system-graph-container");
    const svg = document.getElementById("topic-connections-svg");
    const nodesContainer = document.getElementById("graph-nodes");
    if (!container || !svg || !nodesContainer) return;

    svg.innerHTML = "";

    // --- 1. Parse nodes from DOM ---
    const components = {};
    const events = {};
    const recipeActions = {};

    nodesContainer.querySelectorAll("[data-node-name]").forEach((el) => {
        const name = el.getAttribute("data-node-name");
        components[name] = {
            el,
            pubs: new Set(JSON.parse(el.getAttribute("data-publishers") || "[]").map(normalizeTopic)),
            subs: new Set(JSON.parse(el.getAttribute("data-subscribers") || "[]").map(normalizeTopic)),
            actionName: el.getAttribute("data-action-name") || "",
            srvName: el.getAttribute("data-srv-name") || "",
        };
    });

    nodesContainer.querySelectorAll("[data-node-type='event']").forEach((el) => {
        const id = el.getAttribute("data-event-id");
        events[id] = {
            el,
            involvedTopics: JSON.parse(el.getAttribute("data-involved-topics") || "[]").map(normalizeTopic),
            componentActions: JSON.parse(el.getAttribute("data-component-actions") || "[]"),
            recipeActions: JSON.parse(el.getAttribute("data-recipe-actions") || "[]"),
        };
    });

    nodesContainer.querySelectorAll("[data-node-type='recipe_action']").forEach((el) => {
        recipeActions[el.id] = { el, parentEvent: el.getAttribute("data-parent-event") };
    });

    // --- 2. Build edges & color map ---
    const edges = [];
    let colorIdx = 0;
    const topicColors = {};

    function getColor(topic) {
        if (!topicColors[topic]) {
            topicColors[topic] = EDGE_PALETTE[colorIdx++ % EDGE_PALETTE.length];
        }
        return topicColors[topic];
    }

    // 2a. Component → Component (topic connections)
    const compEdgeKeys = new Set();
    for (const [src, srcD] of Object.entries(components)) {
        for (const topic of srcD.pubs) {
            for (const [dst, dstD] of Object.entries(components)) {
                if (src === dst || !dstD.subs.has(topic)) continue;
                const key = `${src}|${dst}|${topic}`;
                if (compEdgeKeys.has(key)) continue;
                compEdgeKeys.add(key);
                edges.push({ from: src, fromType: "component", to: dst, toType: "component", label: topic, color: getColor(topic), dashed: false });
            }
        }
    }

    // 2b. Component → Event (topic triggers event) with corner assignment
    const eventCorners = {};
    for (const [eventId, evD] of Object.entries(events)) {
        eventCorners[eventId] = {};
        evD.involvedTopics.forEach((topic, idx) => {
            const corner = ENTRY_CORNERS[Math.min(idx, ENTRY_CORNERS.length - 1)];
            eventCorners[eventId][topic] = corner;
            for (const [comp, compD] of Object.entries(components)) {
                if (compD.pubs.has(topic)) {
                    edges.push({ from: comp, fromType: "component", to: eventId, toType: "event", label: topic, color: getColor(topic), dashed: false, entryCorner: corner });
                }
            }
        });

        // 2c. Event → Component (action method call)
        // Use side entry to avoid overlapping with topic edges that enter from top
        for (const a of evD.componentActions) {
            if (a.component && components[a.component]) {
                edges.push({ from: eventId, fromType: "event", to: a.component, toType: "component", label: a.name, color: "#888", dashed: true, entryCorner: "right" });
            }
        }

        // 2d. Event → Recipe action oval
        for (const a of evD.recipeActions) {
            const ovalId = `action-${eventId.slice(0, 8)}-${a.name}`;
            if (recipeActions[ovalId]) {
                edges.push({ from: eventId, fromType: "event", to: ovalId, toType: "recipe_action", label: "", color: "#888", dashed: true });
            }
        }
    }

    // --- 3. Layered layout (topological sort on component-to-component edges) ---
    const allNodes = {};
    const compNames = Object.keys(components);

    // BFS depth assignment
    const inCount = {};
    const outAdj = {};
    compNames.forEach(n => { inCount[n] = 0; outAdj[n] = []; });
    edges.forEach(e => {
        if (e.fromType === "component" && e.toType === "component" && !outAdj[e.from].includes(e.to)) {
            outAdj[e.from].push(e.to);
            inCount[e.to]++;
        }
    });

    const depth = {};
    const queue = compNames.filter(n => inCount[n] === 0);
    queue.forEach(n => { depth[n] = 0; });

    while (queue.length) {
        const cur = queue.shift();
        for (const next of outAdj[cur]) {
            const d = depth[cur] + 1;
            if (depth[next] === undefined || d > depth[next]) depth[next] = d;
            if (--inCount[next] <= 0 && !queue.includes(next)) queue.push(next);
        }
    }
    compNames.forEach(n => { if (depth[n] === undefined) depth[n] = 0; });

    // Assign layers (*2 to leave room for events between component layers)
    compNames.forEach(n => { allNodes[n] = { el: components[n].el, layer: depth[n] * 2 }; });

    let maxLayer = 0;
    Object.values(allNodes).forEach(n => { maxLayer = Math.max(maxLayer, n.layer); });

    // Events: one layer below deepest source component
    for (const [id, evD] of Object.entries(events)) {
        let maxSrc = -1;
        edges.forEach(e => {
            if (e.to === id && e.fromType === "component" && allNodes[e.from]) {
                maxSrc = Math.max(maxSrc, allNodes[e.from].layer);
            }
        });
        allNodes[id] = { el: evD.el, layer: (maxSrc >= 0 ? maxSrc : maxLayer) + 1 };
    }

    // Recipe actions: one layer below parent event
    for (const [id, d] of Object.entries(recipeActions)) {
        const parentLayer = allNodes[d.parentEvent] ? allNodes[d.parentEvent].layer : maxLayer + 1;
        allNodes[id] = { el: d.el, layer: parentLayer + 1 };
    }

    // Group into layers
    const layers = {};
    for (const [id, d] of Object.entries(allNodes)) {
        (layers[d.layer] || (layers[d.layer] = [])).push(id);
    }

    // --- 4. Position nodes (deferred to next frame for accurate measurements) ---
    for (const d of Object.values(allNodes)) {
        d.el.style.position = "absolute";
        d.el.style.visibility = "hidden";
        d.el.style.display = "";
    }

    requestAnimationFrame(() => {
        const containerW = container.offsetWidth;
        let currentY = PADDING_Y;

        Object.keys(layers).map(Number).sort((a, b) => a - b).forEach(layerIdx => {
            const ids = layers[layerIdx];
            const sizes = ids.map(id => ({ id, w: allNodes[id].el.offsetWidth, h: allNodes[id].el.offsetHeight }));
            const totalW = sizes.reduce((s, m) => s + m.w, 0) + NODE_H_GAP * (ids.length - 1);
            let x = Math.max(PADDING_X, (containerW - totalW) / 2);
            let maxH = 0;

            sizes.forEach(({ id, w, h }) => {
                allNodes[id].el.style.left = `${x}px`;
                allNodes[id].el.style.top = `${currentY}px`;
                allNodes[id].el.style.visibility = "visible";
                x += w + NODE_H_GAP;
                maxH = Math.max(maxH, h);
            });
            currentY += maxH + NODE_V_GAP;
        });

        container.style.height = `${currentY + PADDING_Y}px`;

        // Store state for redrawing during drag
        _graphState = { svg, container, allNodes, edges, components, events, eventCorners, topicColors, compNames, getColor };

        // --- 5. Draw edges + stubs ---
        redrawAllEdges();

        // --- 6. Hover + Drag (only on first layout, not re-attached) ---
        if (!container._interactionsAttached) {
            setupHoverHighlighting(allNodes, edges);
            setupDragging(allNodes);
            container._interactionsAttached = true;
        }
    });
}


/** Redraws all edges and stubs based on current node positions. Called after layout and during drag. */
function redrawAllEdges() {
    const s = _graphState;
    if (!s) return;
    const { svg, container, allNodes, edges, components, events, eventCorners, topicColors, compNames, getColor } = s;

    svg.innerHTML = "";

    // Size SVG to container
    svg.setAttribute("width", container.offsetWidth);
    svg.setAttribute("height", container.offsetHeight);
    svg.setAttribute("viewBox", `0 0 ${container.offsetWidth} ${container.offsetHeight}`);

    const cRect = container.getBoundingClientRect();

    // Draw edges
    edges.forEach(e => {
        const fromN = allNodes[e.from];
        const toN = allNodes[e.to];
        if (!fromN || !toN) return;

        const src = getAnchor(fromN.el.getBoundingClientRect(), cRect, "bottom");
        const dst = getAnchor(toN.el.getBoundingClientRect(), cRect, e.entryCorner || "top");
        drawEdge(svg, src, dst, e.entryCorner || "top", e.label, e.color, e.dashed, e.from, e.to);
    });

    // Component external stubs
    const connPubs = {};
    const connSubs = {};
    compNames.forEach(n => { connPubs[n] = new Set(); connSubs[n] = new Set(); });
    edges.forEach(e => {
        if (e.fromType === "component" && e.label) connPubs[e.from].add(normalizeTopic(e.label));
        if (e.toType === "component" && e.label) connSubs[e.to].add(normalizeTopic(e.label));
    });

    for (const [name, data] of Object.entries(components)) {
        const nRect = data.el.getBoundingClientRect();
        const extIn = [...data.subs].filter(t => !connSubs[name].has(t));
        const extOut = [...data.pubs].filter(t => !connPubs[name].has(t));

        if (data.actionName) { extIn.push(normalizeTopic(data.actionName)); topicColors[normalizeTopic(data.actionName)] = "#9B59B6"; }
        if (data.srvName) { extIn.push(normalizeTopic(data.srvName)); topicColors[normalizeTopic(data.srvName)] = "#447AE5"; }

        drawStubRow(svg, cRect, nRect, extIn, topicColors, "input", name);
        drawStubRow(svg, cRect, nRect, extOut, topicColors, "output", name);
    }

    // Event external stubs
    for (const [eventId, evD] of Object.entries(events)) {
        const connTopics = new Set();
        edges.forEach(e => { if (e.to === eventId && e.fromType === "component") connTopics.add(normalizeTopic(e.label)); });
        const extTopics = evD.involvedTopics.filter(t => !connTopics.has(t));
        const corners = eventCorners[eventId] || {};
        const nRect = evD.el.getBoundingClientRect();

        extTopics.forEach(topic => {
            const corner = corners[topic] || "top";
            const anchor = getAnchor(nRect, cRect, corner);
            const dir = corner === "left" ? "from-left" : corner === "right" ? "from-right" : "from-top";
            drawStub(svg, anchor.x, anchor.y, dir, topic, topicColors[topic] || getColor(topic), eventId);
        });
    }
}

/** Draw a row of stubs spaced horizontally along the top or bottom of a node.
 *  Stubs fan out for label spacing but arrows always touch the node edge. */
function drawStubRow(svg, cRect, nRect, topics, topicColors, direction, nodeId) {
    if (!topics.length) return;

    const stubLen = 30;
    const as = 4;
    const labelWidths = topics.map(t => (t.startsWith("/") ? t.slice(1) : t).length * 7);
    const spacing = Math.max(50, ...labelWidths);
    const totalW = topics.length * spacing;
    const outerStartX = nRect.left + (nRect.width - totalW) / 2 - cRect.left + spacing / 2;

    // Distribute arrow endpoints evenly across the node width
    const nodeLeft = nRect.left - cRect.left;
    const nodeW = nRect.width;
    const edgeY = direction === "input" ? (nRect.top - cRect.top) : (nRect.bottom - cRect.top);

    topics.forEach((topic, idx) => {
        const color = topicColors[topic] || EDGE_PALETTE[idx % EDGE_PALETTE.length];
        if (!topicColors[topic]) topicColors[topic] = color;
        const shortName = topic.startsWith("/") ? topic.slice(1) : topic;

        // Outer point (where label sits) — fanned out for spacing
        const outerX = outerStartX + idx * spacing;
        // Inner point (arrow tip) — distributed within the node width
        const innerX = topics.length === 1
            ? nodeLeft + nodeW / 2
            : nodeLeft + nodeW * 0.2 + (nodeW * 0.6) * (idx / (topics.length - 1));

        let outerY, innerY, labelY;
        if (direction === "input") {
            outerY = edgeY - stubLen;
            innerY = edgeY;
            labelY = outerY - 5;
        } else {
            outerY = edgeY + stubLen;
            innerY = edgeY;
            labelY = outerY + 12;
        }

        // Arrow tip and tail points
        const tipX = direction === "input" ? innerX : outerX;
        const tipY = direction === "input" ? innerY : outerY;
        const tailX = direction === "input" ? outerX : innerX;
        const tailY = direction === "input" ? outerY : innerY;

        // Compute arrowhead aligned to the line direction
        const angle = Math.atan2(tipY - tailY, tipX - tailX);
        const ax1 = tipX - as * 1.5 * Math.cos(angle - 0.4);
        const ay1 = tipY - as * 1.5 * Math.sin(angle - 0.4);
        const ax2 = tipX - as * 1.5 * Math.cos(angle + 0.4);
        const ay2 = tipY - as * 1.5 * Math.sin(angle + 0.4);
        const arrowPts = `${tipX},${tipY} ${ax1},${ay1} ${ax2},${ay2}`;

        const dataAttr = { "data-node": nodeId || "" };

        // Line from outer point to inner point
        svg.appendChild(svgEl("line", {
            x1: outerX, y1: outerY, x2: innerX, y2: innerY,
            class: "topic-stub", ...dataAttr, style: { stroke: color },
        }));

        // Arrow (rotated to match line angle)
        svg.appendChild(svgEl("polygon", {
            points: arrowPts, class: "topic-stub-arrow", ...dataAttr, style: { fill: color },
        }));

        // Label at outer end
        const text = svgEl("text", {
            x: outerX, y: labelY, class: "topic-stub-label", "text-anchor": "middle",
            ...dataAttr, style: { fill: color },
        });
        text.textContent = shortName;
        svg.appendChild(text);
    });
}


// ─── Dragging ─────────────────────────────────────────────────

function setupDragging(allNodes) {
    let dragNode = null;
    let dragOffsetX = 0;
    let dragOffsetY = 0;
    let rafPending = false;

    function onPointerDown(e, nodeId) {
        // Ignore clicks on interactive children (buttons, links)
        if (["BUTTON", "A", "INPUT"].includes(e.target.tagName)) return;

        const el = allNodes[nodeId].el;
        const rect = el.getBoundingClientRect();
        dragOffsetX = (e.clientX || e.touches[0].clientX) - rect.left;
        dragOffsetY = (e.clientY || e.touches[0].clientY) - rect.top;
        dragNode = { id: nodeId, el };

        el.style.zIndex = "10";
        el.style.cursor = "grabbing";
        document.body.style.userSelect = "none";

        if (e.cancelable) e.preventDefault();
    }

    function onPointerMove(e) {
        if (!dragNode) return;
        if (e.cancelable) e.preventDefault();

        const container = _graphState.container;
        const cRect = container.getBoundingClientRect();
        const clientX = e.clientX || (e.touches && e.touches[0].clientX) || 0;
        const clientY = e.clientY || (e.touches && e.touches[0].clientY) || 0;

        const newLeft = clientX - cRect.left - dragOffsetX;
        const newTop = clientY - cRect.top - dragOffsetY;

        dragNode.el.style.left = `${newLeft}px`;
        dragNode.el.style.top = `${newTop}px`;

        // Throttle edge redraw to animation frames
        if (!rafPending) {
            rafPending = true;
            requestAnimationFrame(() => {
                redrawAllEdges();
                rafPending = false;
            });
        }
    }

    function onPointerUp() {
        if (!dragNode) return;
        dragNode.el.style.zIndex = "2";
        dragNode.el.style.cursor = "";
        document.body.style.userSelect = "";

        // Mark that the user has manually arranged nodes
        _userDragged = true;

        // Expand container if node was dragged below current bottom
        const container = _graphState.container;
        const el = dragNode.el;
        const bottom = el.offsetTop + el.offsetHeight + PADDING_Y;
        if (bottom > container.offsetHeight) {
            container.style.height = `${bottom}px`;
        }

        dragNode = null;
        redrawAllEdges();
    }

    // Attach per-node handlers
    for (const [nodeId, nodeData] of Object.entries(allNodes)) {
        const el = nodeData.el;
        el.style.cursor = "grab";
        el.addEventListener("mousedown", (e) => onPointerDown(e, nodeId));
        el.addEventListener("touchstart", (e) => onPointerDown(e, nodeId), { passive: false });
    }

    // Global move/up handlers
    document.addEventListener("mousemove", onPointerMove);
    document.addEventListener("mouseup", onPointerUp);
    document.addEventListener("touchmove", onPointerMove, { passive: false });
    document.addEventListener("touchend", onPointerUp);
}


// ─── Hover ────────────────────────────────────────────────────

function setupHoverHighlighting(allNodes, edges) {
    for (const [nodeId, nodeData] of Object.entries(allNodes)) {
        nodeData.el.addEventListener("mouseenter", () => {
            const connected = new Set([nodeId]);
            edges.forEach(e => {
                if (e.from === nodeId) connected.add(e.to);
                if (e.to === nodeId) connected.add(e.from);
            });

            // Dim unconnected nodes
            for (const [id, d] of Object.entries(allNodes)) {
                if (!connected.has(id)) d.el.classList.add("dimmed");
            }

            // Highlight/dim SVG elements
            svg_highlight(connected, nodeId);
        });

        nodeData.el.addEventListener("mouseleave", () => {
            for (const d of Object.values(allNodes)) d.el.classList.remove("dimmed");
            svg_reset();
        });
    }
}

function svg_highlight(connectedNodes, hoveredId) {
    document.querySelectorAll(".topic-line, .topic-arrow").forEach(el => {
        if (el.getAttribute("data-from") === hoveredId || el.getAttribute("data-to") === hoveredId) {
            el.classList.add("highlighted");
        } else {
            el.style.opacity = "0.08";
        }
    });
    document.querySelectorAll(".topic-label").forEach(el => {
        if (el.getAttribute("data-from") === hoveredId || el.getAttribute("data-to") === hoveredId) {
            el.style.opacity = "1";
            el.style.fontWeight = "600";
        } else {
            el.style.opacity = "0.08";
        }
    });
    document.querySelectorAll(".topic-stub, .topic-stub-arrow, .topic-stub-label").forEach(el => {
        const n = el.getAttribute("data-node");
        el.style.opacity = (n && connectedNodes.has(n)) ? "1" : "0.08";
    });
}

function svg_reset() {
    document.querySelectorAll(".topic-line, .topic-arrow").forEach(el => {
        el.classList.remove("highlighted");
        el.style.opacity = "";
    });
    document.querySelectorAll(".topic-label").forEach(el => {
        el.style.opacity = "";
        el.style.fontWeight = "";
    });
    document.querySelectorAll(".topic-stub, .topic-stub-arrow, .topic-stub-label").forEach(el => {
        el.style.opacity = "";
    });
}


// ─── Auto-init ────────────────────────────────────────────────

document.addEventListener("DOMContentLoaded", () => {
    const resizeObs = new ResizeObserver(() => {
        if (document.getElementById("system-graph-container")) drawTopicConnections();
    });

    const existing = document.getElementById("system-graph-container");
    if (existing) { resizeObs.observe(existing); drawTopicConnections(); }

    new MutationObserver(() => {
        const c = document.getElementById("system-graph-container");
        if (c && !c._observed) {
            resizeObs.observe(c);
            c._observed = true;
            // Fresh DOM — reset user drag state so layout is recomputed
            _userDragged = false;
            _graphState = null;
            setTimeout(drawTopicConnections, 100);
        }
    }).observe(document.body, { childList: true, subtree: true });
});
