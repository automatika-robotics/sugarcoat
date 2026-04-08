/**
 * system_graph.js
 * Lays out component nodes in a layered graph and draws SVG edges between them.
 * Nodes are positioned absolutely; edges are drawn as smooth bezier curves.
 */

// Layout constants
const NODE_H_GAP = 60;   // Horizontal gap between nodes in the same layer
const NODE_V_GAP = 120;  // Vertical gap between layers (room for edge labels)
const PADDING_X = 40;
const PADDING_Y = 60;   // Room for external input stubs above first layer

// Color palette for edges
const EDGE_PALETTE = [
    "#E83F3F", "#447AE5", "#2ECC71", "#9B59B6", "#F1C40F",
    "#E67E22", "#1ABC9C", "#FF69B4", "#00CED1", "#FF6347",
];


/**
 * Main entry point: computes layout, positions nodes, draws edges.
 */
function drawTopicConnections() {
    const container = document.getElementById("system-graph-container");
    const svg = document.getElementById("topic-connections-svg");
    const row = document.getElementById("components-row");
    if (!container || !svg || !row) return;

    svg.innerHTML = "";

    const nodes = Array.from(row.querySelectorAll("[data-node-name]"));
    if (nodes.length === 0) return;

    // --- 1. Parse node data ---
    // Normalize topic names: strip leading slash for consistent matching
    function normalizeTopic(t) {
        return t.startsWith("/") ? t.slice(1) : t;
    }

    const nodeMap = {};
    nodes.forEach((el) => {
        const name = el.getAttribute("data-node-name");
        const rawPubs = JSON.parse(el.getAttribute("data-publishers") || "[]");
        const rawSubs = JSON.parse(el.getAttribute("data-subscribers") || "[]");
        const pubs = new Set(rawPubs.map(normalizeTopic));
        const subs = new Set(rawSubs.map(normalizeTopic));
        // Action/service interface names (shown as stubs for action servers / servers)
        const actionName = el.getAttribute("data-action-name") || "";
        const srvName = el.getAttribute("data-srv-name") || "";
        nodeMap[name] = { el, pubs, subs, rawPubs, rawSubs, actionName, srvName };
    });

    // --- 2. Build directed edges (publisher → subscriber) ---
    const edges = [];       // { from, to, topic }
    const topicColors = {};
    let colorIdx = 0;
    const edgeSet = new Set();  // deduplicate

    for (const [srcName, srcData] of Object.entries(nodeMap)) {
        for (const topic of srcData.pubs) {
            for (const [dstName, dstData] of Object.entries(nodeMap)) {
                if (srcName === dstName) continue;
                if (dstData.subs.has(topic)) {
                    const key = `${srcName}|${dstName}|${topic}`;
                    if (edgeSet.has(key)) continue;
                    edgeSet.add(key);
                    if (!topicColors[topic]) {
                        topicColors[topic] = EDGE_PALETTE[colorIdx % EDGE_PALETTE.length];
                        colorIdx++;
                    }
                    edges.push({ from: srcName, to: dstName, topic });
                }
            }
        }
    }

    // --- 3. Compute layered layout (topological sort by depth) ---
    const layers = computeLayers(nodeMap, edges);

    // --- 4. Make nodes visible but hidden so browser can compute sizes ---
    for (const name of Object.keys(nodeMap)) {
        const el = nodeMap[name].el;
        el.style.position = "absolute";
        el.style.visibility = "hidden";
        el.style.display = "block";
    }

    // --- 5. Defer layout and drawing to next frame so the browser has computed element sizes ---
    requestAnimationFrame(() => {
    positionNodesSync(container, nodeMap, layers);

    // --- 6. Size the SVG to match container ---
    const containerH = container.offsetHeight;
    const containerW = container.offsetWidth;
    svg.setAttribute("width", containerW);
    svg.setAttribute("height", containerH);
    svg.setAttribute("viewBox", `0 0 ${containerW} ${containerH}`);

    // --- 7. Draw edges ---
    edges.forEach((edge) => {
        drawEdge(svg, container, nodeMap[edge.from].el, nodeMap[edge.to].el, edge.topic, topicColors[edge.topic]);
    });

    // --- 8. Draw unconnected (external) topic stubs ---
    // Find topics that are subscribed but not published by any node in the graph (external inputs)
    // and topics published but not subscribed by any node (external outputs)
    const allInternalPubs = new Set();
    const allInternalSubs = new Set();
    edges.forEach((e) => {
        allInternalPubs.add(e.topic + "|" + e.from);
        allInternalSubs.add(e.topic + "|" + e.to);
    });

    // Build per-node sets of topics that are connected by an edge
    // A subscription is "connected" if there's an edge delivering that topic TO this node
    // A publication is "connected" if there's an edge carrying that topic FROM this node
    const connectedSubsPerNode = {};  // nodeName -> Set of topics received via edges
    const connectedPubsPerNode = {};  // nodeName -> Set of topics sent via edges
    for (const name of Object.keys(nodeMap)) {
        connectedSubsPerNode[name] = new Set();
        connectedPubsPerNode[name] = new Set();
    }
    edges.forEach((e) => {
        connectedPubsPerNode[e.from].add(e.topic);
        connectedSubsPerNode[e.to].add(e.topic);
    });

    for (const [name, data] of Object.entries(nodeMap)) {
        // External inputs: subscribed topics with no internal publisher delivering to this node
        const externalInputs = [];
        for (const topic of data.subs) {
            if (!connectedSubsPerNode[name].has(topic)) {
                if (!topicColors[topic]) {
                    topicColors[topic] = EDGE_PALETTE[colorIdx % EDGE_PALETTE.length];
                    colorIdx++;
                }
                externalInputs.push(topic);
            }
        }

        // External outputs: published topics with no internal subscriber receiving from this node
        const externalOutputs = [];
        for (const topic of data.pubs) {
            if (!connectedPubsPerNode[name].has(topic)) {
                if (!topicColors[topic]) {
                    topicColors[topic] = EDGE_PALETTE[colorIdx % EDGE_PALETTE.length];
                    colorIdx++;
                }
                externalOutputs.push(topic);
            }
        }

        // Add action/service interface as an input stub for action servers / servers
        if (data.actionName) {
            const aName = normalizeTopic(data.actionName);
            if (!topicColors[aName]) {
                topicColors[aName] = "#9B59B6";  // Purple for actions
            }
            externalInputs.push(aName);
        }
        if (data.srvName) {
            const sName = normalizeTopic(data.srvName);
            if (!topicColors[sName]) {
                topicColors[sName] = "#447AE5";  // Blue for services
            }
            externalInputs.push(sName);
        }

        if (externalInputs.length > 0) {
            drawExternalStubs(svg, container, data.el, externalInputs, topicColors, "input");
        }
        if (externalOutputs.length > 0) {
            drawExternalStubs(svg, container, data.el, externalOutputs, topicColors, "output");
        }
    }

    // --- 9. Hover interactions ---
    setupHoverHighlighting(nodes, edges);

    }); // end requestAnimationFrame
}


/**
 * Simple layered assignment: nodes with no incoming edges are layer 0,
 * their dependents are layer 1, etc. Nodes with no edges go to layer 0.
 */
function computeLayers(nodeMap, edges) {
    const names = Object.keys(nodeMap);
    const incomingCount = {};
    const outgoing = {};   // name -> [name, ...]

    names.forEach((n) => {
        incomingCount[n] = 0;
        outgoing[n] = [];
    });

    edges.forEach((e) => {
        // Deduplicate: only count unique from→to pairs
        if (!outgoing[e.from].includes(e.to)) {
            outgoing[e.from].push(e.to);
            incomingCount[e.to] = (incomingCount[e.to] || 0) + 1;
        }
    });

    // BFS-based layer assignment
    const depth = {};
    const queue = [];

    names.forEach((n) => {
        if (incomingCount[n] === 0) {
            depth[n] = 0;
            queue.push(n);
        }
    });

    while (queue.length > 0) {
        const current = queue.shift();
        for (const next of outgoing[current]) {
            const newDepth = depth[current] + 1;
            if (depth[next] === undefined || newDepth > depth[next]) {
                depth[next] = newDepth;
            }
            incomingCount[next]--;
            if (incomingCount[next] <= 0 && !queue.includes(next)) {
                queue.push(next);
            }
        }
    }

    // Assign any remaining (cyclic or isolated) nodes
    names.forEach((n) => {
        if (depth[n] === undefined) depth[n] = 0;
    });

    // Group by layer
    const layers = {};
    for (const [name, d] of Object.entries(depth)) {
        if (!layers[d]) layers[d] = [];
        layers[d].push(name);
    }

    return layers;
}


/**
 * Positions nodes in a top-to-bottom layered layout, centered horizontally.
 * Assumes nodes are already in the DOM with position:absolute and visibility:hidden
 * so that offsetWidth/offsetHeight are valid.
 */
function positionNodesSync(container, nodeMap, layers) {
    const layerKeys = Object.keys(layers).map(Number).sort((a, b) => a - b);

    // Compute positions
    const containerWidth = container.offsetWidth;
    let currentY = PADDING_Y;
    let maxRight = 0;

    layerKeys.forEach((layerIdx) => {
        const layerNodes = layers[layerIdx];

        // Measure total width of this layer
        let totalWidth = 0;
        const measurements = [];
        layerNodes.forEach((name) => {
            const el = nodeMap[name].el;
            const w = el.offsetWidth;
            const h = el.offsetHeight;
            measurements.push({ name, w, h });
            totalWidth += w;
        });
        totalWidth += NODE_H_GAP * (layerNodes.length - 1);

        // Center the layer horizontally
        let startX = Math.max(PADDING_X, (containerWidth - totalWidth) / 2);
        let maxH = 0;

        measurements.forEach(({ name, w, h }) => {
            const el = nodeMap[name].el;
            el.style.left = `${startX}px`;
            el.style.top = `${currentY}px`;
            el.style.visibility = "visible";
            startX += w + NODE_H_GAP;
            maxH = Math.max(maxH, h);
            maxRight = Math.max(maxRight, startX);
        });

        currentY += maxH + NODE_V_GAP;
    });

    // Set container height to fit all layers + room for output stubs below
    container.style.height = `${currentY + PADDING_Y + 30}px`;
}


/**
 * Draws a smooth bezier edge from the bottom of srcEl to the top of dstEl.
 */
function drawEdge(svg, container, srcEl, dstEl, topicName, color) {
    const cRect = container.getBoundingClientRect();

    const srcRect = srcEl.getBoundingClientRect();
    const dstRect = dstEl.getBoundingClientRect();

    // Start from bottom-center of source
    const x1 = srcRect.left + srcRect.width / 2 - cRect.left;
    const y1 = srcRect.bottom - cRect.top;

    // End at top-center of destination
    const x2 = dstRect.left + dstRect.width / 2 - cRect.left;
    const y2 = dstRect.top - cRect.top;

    const dy = Math.abs(y2 - y1);
    const curvature = Math.max(dy * 0.4, 40);

    // Bezier path
    const path = document.createElementNS("http://www.w3.org/2000/svg", "path");
    const d = `M ${x1} ${y1} C ${x1} ${y1 + curvature}, ${x2} ${y2 - curvature}, ${x2} ${y2}`;
    path.setAttribute("d", d);
    path.setAttribute("class", "topic-line");
    path.setAttribute("data-topic", topicName);
    path.setAttribute("data-from", srcEl.getAttribute("data-node-name"));
    path.setAttribute("data-to", dstEl.getAttribute("data-node-name"));
    path.style.stroke = color;
    svg.appendChild(path);

    // Arrow head at destination
    const arrowSize = 6;
    const arrow = document.createElementNS("http://www.w3.org/2000/svg", "polygon");
    arrow.setAttribute("points", `${x2},${y2} ${x2 - arrowSize},${y2 - arrowSize * 1.5} ${x2 + arrowSize},${y2 - arrowSize * 1.5}`);
    arrow.setAttribute("class", "topic-arrow");
    arrow.setAttribute("data-topic", topicName);
    arrow.setAttribute("data-from", srcEl.getAttribute("data-node-name"));
    arrow.setAttribute("data-to", dstEl.getAttribute("data-node-name"));
    arrow.style.fill = color;
    svg.appendChild(arrow);

    // Topic label at midpoint
    const midX = (x1 + x2) / 2;
    const midY = (y1 + y2) / 2;

    const label = document.createElementNS("http://www.w3.org/2000/svg", "text");
    label.setAttribute("x", midX);
    label.setAttribute("y", midY);
    label.setAttribute("class", "topic-label");
    label.setAttribute("text-anchor", "middle");
    label.setAttribute("data-topic", topicName);
    label.setAttribute("data-from", srcEl.getAttribute("data-node-name"));
    label.setAttribute("data-to", dstEl.getAttribute("data-node-name"));
    label.style.fill = color;

    const shortName = topicName.startsWith("/") ? topicName.slice(1) : topicName;
    label.textContent = shortName;
    svg.appendChild(label);
}


/**
 * Draws small stub lines with labels for topics that have no connection to another node in the graph.
 * Inputs arrive from the top of the node, outputs exit from the bottom.
 *
 * @param {SVGElement} svg - The SVG overlay element
 * @param {HTMLElement} container - The graph container
 * @param {HTMLElement} nodeEl - The component card element
 * @param {string[]} topics - List of unconnected topic names
 * @param {Object} topicColors - Topic name → color mapping
 * @param {"input"|"output"} direction - Whether these are inputs or outputs
 */
function drawExternalStubs(svg, container, nodeEl, topics, topicColors, direction) {
    const cRect = container.getBoundingClientRect();
    const nRect = nodeEl.getBoundingClientRect();
    const nodeName = nodeEl.getAttribute("data-node-name");

    const stubLength = 30;

    // Measure label widths to compute proper spacing
    // Use a heuristic: ~7px per character at 0.6rem font size
    const labelWidths = topics.map(t => {
        const name = t.startsWith("/") ? t.slice(1) : t;
        return name.length * 7;
    });
    const spacing = Math.max(50, ...labelWidths);

    // Space stubs horizontally across the top or bottom edge of the node
    const totalWidth = topics.length * spacing;
    const startX = nRect.left + (nRect.width - totalWidth) / 2 - cRect.left + spacing / 2;

    topics.forEach((topic, idx) => {
        const color = topicColors[topic] || "#888";
        const shortName = topic.startsWith("/") ? topic.slice(1) : topic;
        const x = startX + idx * spacing;

        let x1, y1, x2, y2, labelY, arrowPoints;

        if (direction === "input") {
            // Stub comes from above into the top of the node
            y2 = nRect.top - cRect.top;
            y1 = y2 - stubLength;
            x1 = x;
            x2 = x;
            labelY = y1 - 5;
            // Arrow pointing down into the node
            const as = 4;
            arrowPoints = `${x2},${y2} ${x2 - as},${y2 - as * 1.5} ${x2 + as},${y2 - as * 1.5}`;
        } else {
            // Stub exits from the bottom of the node downward
            y1 = nRect.bottom - cRect.top;
            y2 = y1 + stubLength;
            x1 = x;
            x2 = x;
            labelY = y2 + 12;
            // Arrow pointing down away from the node
            const as = 4;
            arrowPoints = `${x2},${y2} ${x2 - as},${y2 - as * 1.5} ${x2 + as},${y2 - as * 1.5}`;
        }

        // Stub line
        const line = document.createElementNS("http://www.w3.org/2000/svg", "line");
        line.setAttribute("x1", x1);
        line.setAttribute("y1", y1);
        line.setAttribute("x2", x2);
        line.setAttribute("y2", y2);
        line.setAttribute("class", "topic-stub");
        line.setAttribute("data-node", nodeName);
        line.style.stroke = color;
        svg.appendChild(line);

        // Arrow
        const arrow = document.createElementNS("http://www.w3.org/2000/svg", "polygon");
        arrow.setAttribute("points", arrowPoints);
        arrow.setAttribute("class", "topic-stub-arrow");
        arrow.setAttribute("data-node", nodeName);
        arrow.style.fill = color;
        svg.appendChild(arrow);

        // Label (rotated for vertical stubs)
        const label = document.createElementNS("http://www.w3.org/2000/svg", "text");
        label.setAttribute("x", x);
        label.setAttribute("y", labelY);
        label.setAttribute("class", "topic-stub-label");
        label.setAttribute("text-anchor", "middle");
        label.setAttribute("data-node", nodeName);
        label.style.fill = color;
        label.textContent = shortName;
        svg.appendChild(label);
    });
}


/**
 * On hover: highlights connected edges, dims unrelated nodes.
 */
function setupHoverHighlighting(nodeElements, edges) {
    nodeElements.forEach((node) => {
        const nodeName = node.getAttribute("data-node-name");

        node.addEventListener("mouseenter", () => {
            // Find connected node names
            const connected = new Set([nodeName]);
            edges.forEach((e) => {
                if (e.from === nodeName) connected.add(e.to);
                if (e.to === nodeName) connected.add(e.from);
            });

            // Dim unconnected nodes
            nodeElements.forEach((n) => {
                if (!connected.has(n.getAttribute("data-node-name"))) {
                    n.classList.add("dimmed");
                }
            });

            // Highlight connected edges, dim others
            document.querySelectorAll(".topic-line, .topic-arrow").forEach((el) => {
                if (el.getAttribute("data-from") === nodeName || el.getAttribute("data-to") === nodeName) {
                    el.classList.add("highlighted");
                } else {
                    el.style.opacity = "0.08";
                }
            });

            document.querySelectorAll(".topic-label").forEach((el) => {
                if (el.getAttribute("data-from") === nodeName || el.getAttribute("data-to") === nodeName) {
                    el.style.opacity = "1";
                    el.style.fontWeight = "600";
                } else {
                    el.style.opacity = "0.08";
                }
            });

            // Dim/highlight external stubs
            document.querySelectorAll(".topic-stub, .topic-stub-arrow, .topic-stub-label").forEach((el) => {
                if (el.getAttribute("data-node") === nodeName) {
                    el.style.opacity = "1";
                } else {
                    el.style.opacity = "0.08";
                }
            });
        });

        node.addEventListener("mouseleave", () => {
            nodeElements.forEach((n) => n.classList.remove("dimmed"));

            document.querySelectorAll(".topic-line, .topic-arrow").forEach((el) => {
                el.classList.remove("highlighted");
                el.style.opacity = "";
            });

            document.querySelectorAll(".topic-label").forEach((el) => {
                el.style.opacity = "";
                el.style.fontWeight = "";
            });

            document.querySelectorAll(".topic-stub, .topic-stub-arrow, .topic-stub-label").forEach((el) => {
                el.style.opacity = "";
            });
        });
    });
}


// --- Auto-initialization ---
document.addEventListener("DOMContentLoaded", () => {
    const observer = new ResizeObserver(() => {
        if (document.getElementById("system-graph-container")) {
            drawTopicConnections();
        }
    });

    const existing = document.getElementById("system-graph-container");
    if (existing) {
        observer.observe(existing);
        drawTopicConnections();
    }

    // Watch for the container being added via HTMX swap
    const bodyObserver = new MutationObserver(() => {
        const container = document.getElementById("system-graph-container");
        if (container && !container._resizeObserved) {
            observer.observe(container);
            container._resizeObserved = true;
            setTimeout(drawTopicConnections, 100);
        }
    });
    bodyObserver.observe(document.body, { childList: true, subtree: true });
});
