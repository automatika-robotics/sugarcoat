// Establish WebSocket connection for audio transmission (protocol aware)
const wsProtocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
const ws_stream = new WebSocket(`${wsProtocol}//${window.location.host}/ws_audio`);
ws_stream.onopen = () => {
    console.log("Audio Websocket connection established");
};

ws_stream.onclose = () => {
    console.log("Audio Websocket connection closed");
};

ws_stream.onerror = (err) => {
    console.error(`Audio WebSocket error:`, err);
};


document.addEventListener("DOMContentLoaded", () => {
    const wsProtocol = window.location.protocol === "https:" ? "wss:" : "ws:";
    const MAX_RETRIES = 10; // reconnect retries
    const wsConnections = new Map(); // id -> { ws, timer }

    // Create connection for a given <img> element
    function connectImageWebSocket(img, wsUrl, attempt = 0) {
        const ws = new WebSocket(wsUrl);
        ws.binaryType = "arraybuffer";

        ws.onopen = () => {
            console.log(`[${img.id}] WebSocket connected`);
            attempt = 0;
        };

        ws.onmessage = (event) => {
            try {
                const data = JSON.parse(event.data);
                if (data.payload) {
                    img.src = "data:image/jpeg;base64," + data.payload;
                    return;
                }
            } catch {
                // Not JSON / not base64 JSON; continue to check binary
            }

            if (event.data instanceof ArrayBuffer) {
                const bytes = new Uint8Array(event.data);
                let binary = '';
                bytes.forEach(b => binary += String.fromCharCode(b));
                img.src = "data:image/jpeg;base64," + btoa(binary);
            }
        };

        ws.onerror = (err) => {
            console.error(`[${img.id}] WebSocket error:`, err);
        };

        ws.onclose = () => {
            console.warn(`⚠️ [${img.id}] WebSocket closed`);
            // Clean current entry (but keep any pending reconnect timer)
            wsConnections.delete(img.id);

            if (attempt < MAX_RETRIES) {
                const delay = Math.min(1000 * Math.pow(2, attempt), 30000);
                console.log(`[${img.id}] Reconnecting in ${delay / 1000}s...`);
                const timer = setTimeout(() => {
                    // Only reconnect if the <img> is still present in DOM
                    const stillThere = document.getElementById(img.id);
                    if (stillThere && stillThere.getAttribute("name") === "video-frame") {
                        const conn = connectImageWebSocket(stillThere, wsUrl, attempt + 1);
                        wsConnections.set(img.id, conn);
                    } else {
                        console.log(`[${img.id}] Element no longer present; aborting reconnect.`);
                    }
                }, delay);
                wsConnections.set(img.id, { ws: null, timer });
            } else {
                console.error(`[${img.id}] Max reconnect attempts reached`);
            }
        };

        return { ws };
    }

    // Start connections for all current frames (only if not already tracked)
    // Called from main page with htmx hooks
    function ensureConnectionsForPresentFrames() {
        const frames = Array.from(document.getElementsByName("video-frame"));
        const presentIds = new Set(frames.map(f => f.id));

        // Add missing connections
        frames.forEach((img) => {
            if (!img.id) return; // skip images without id
            if (!wsConnections.has(img.id)) {
                const wsUrl = `${wsProtocol}//${window.location.host}/ws_${img.id}`;
                const conn = connectImageWebSocket(img, wsUrl, 0);
                wsConnections.set(img.id, conn);
            }
        });

        // Remove connections whose elements are gone
        Array.from(wsConnections.keys()).forEach((id) => {
            if (!presentIds.has(id)) {
                const entry = wsConnections.get(id);
                if (entry) {
                    try {
                        if (entry.timer) clearTimeout(entry.timer);
                        if (entry.ws) {
                            entry.ws.onclose = null;
                            entry.ws.close();
                        }
                    } catch (e) {}
                }
                wsConnections.delete(id);
                console.log(`[${id}] Connection removed because element is no longer present`);
            }
        });
    }

    // Close all connections (used on hide/unload)
    function closeAllConnections() {
        wsConnections.forEach((entry, _) => {
            try {
                if (entry.timer) clearTimeout(entry.timer);
                if (entry.ws) {
                    entry.ws.onclose = null;
                    entry.ws.close();
                }
            } catch (e) {}
        });
        wsConnections.clear();
    }

    // MutationObserver fallback: watch for DOM changes to keep connections in sync.
    // We debounce rapid changes.
    let moTimer = null;
    const mo = new MutationObserver(() => {
        if (moTimer) clearTimeout(moTimer);
        moTimer = setTimeout(() => {
            ensureConnectionsForPresentFrames();
        }, 100); // 100ms debounce
    });
    mo.observe(document.body, { childList: true, subtree: true, attributes: false });

    // Visibility and bfcache handling
    document.addEventListener("visibilitychange", () => {
        if (document.visibilityState === "visible") {
            console.log("Page visible, ensuring streams...");
            ensureConnectionsForPresentFrames();
        } else {
            console.log("Page hidden, closing streams...");
            closeAllConnections();
        }
    });

    window.addEventListener("pageshow", (event) => {
        if (event.persisted) {
            console.log("Page restored from bfcache, ensuring streams...");
            ensureConnectionsForPresentFrames();
        }
    });

    window.addEventListener("pagehide", () => {
        closeAllConnections();
    });

    // Initial pass
    ensureConnectionsForPresentFrames();
});

// Fix parsing boolean values from UI Switch elements
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

// Function to make all elements with class "draggable" movable
let highestZ = 1; // Global tracker for z-index of draggable items
function makeDraggable(className = "draggable") {
    const elements = Array.from(document.getElementsByClassName(className));

    elements.forEach(el => {
        let isDragging = false;
        let offsetX = 0, offsetY = 0;

        // Helper function to start dragging
        const startDrag = (clientX, clientY, fixedH = false) => {
            isDragging = true;

            // Bring this element to the top
            highestZ++;
            el.style.zIndex = highestZ;

            // Preserve size
            const rect = el.getBoundingClientRect();
            const computedStyle = window.getComputedStyle(el);
            el.style.width = computedStyle.width;
            if (fixedH) {
                el.style.height = computedStyle.height;
            }
            else {
                el.style.height = "auto";
            }
            el.style.maxHeight = "60vh"; // fallback max height

            // Set absolute position
            el.style.position = "absolute";
            el.style.left = `${rect.left}px`;
            el.style.top = `${rect.top}px`;
            el.style.transition = "none";

            // Calculate offset between mouse/touch and element top-left
            offsetX = clientX - rect.left;
            offsetY = clientY - rect.top;
        };

        // Mouse events
        el.addEventListener("mousedown", (e) => {
            // Ignore dragging if clicking inside:
            // - Elements with class "no-drag"
            // - Form fields: input, textarea, select, button
            if (
                e.target.closest(".no-drag") ||
                e.target.tagName === "INPUT" ||
                e.target.tagName === "TEXTAREA" ||
                e.target.tagName === "SELECT" ||
                e.target.tagName === "BUTTON"
            ) return;


            let fixH = false;
            if (e.target.closest(".fix-size")) fixH = true;

            startDrag(e.clientX, e.clientY, fixH);
        });

        document.addEventListener("mousemove", (e) => {
            if (!isDragging) return;
            el.style.left = `${e.clientX - offsetX}px`;
            el.style.top = `${e.clientY - offsetY}px`;
        });
        document.addEventListener("mouseup", () => {
            if (isDragging) {
                isDragging = false;
                el.style.transition = "";
            }
        });

        // Touch events for mobile
        el.addEventListener("touchstart", (e) => {
            const touch = e.touches[0];
            startDrag(touch.clientX, touch.clientY);
        });
        document.addEventListener("touchmove", (e) => {
            if (!isDragging) return;
            const touch = e.touches[0];
            el.style.left = `${touch.clientX - offsetX}px`;
            el.style.top = `${touch.clientY - offsetY}px`;
        }, { passive: false }); // Prevent scrolling while dragging
        document.addEventListener("touchend", () => {
            isDragging = false;
        });
    });
}


document.addEventListener("DOMContentLoaded", () => {
    makeDraggable();
});


// Audio Recording Logic
let mediaRecorder;
let audioChunks = [];
let isRecording = false;
let recordingIndicatorEl = null; // reference to the indicator message in DOM

function presistForm(form) {
    FormPersistence.persist(form);
}

async function startAudioRecording(button) {
    uk_icon_recording = `<uk-icon icon="mic"><!----><svg xmlns="http://www.w3.org/2000/svg" width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round" class=""><path d="M12 2a3 3 0 0 0-3 3v7a3 3 0 0 0 6 0V5a3 3 0 0 0-3-3Z"></path><path d="M19 10v2a7 7 0 0 1-14 0v-2"></path><line x1="12" x2="12" y1="19" y2="22"></line></svg></uk-icon>`;
    uk_icon_stop = `<uk-icon icon="square"><!----><svg xmlns="http://www.w3.org/2000/svg" width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round" class=""><rect width="18" height="18" x="3" y="3" rx="2"></rect></svg></uk-icon>`;

    button.classList.toggle("recording");

    if (isRecording) {
        mediaRecorder.stop();
        button.innerHTML = uk_icon_recording;
    } else {
        try {
            const stream = await navigator.mediaDevices.getUserMedia({ audio: true });
            mediaRecorder = new MediaRecorder(stream);
            audioChunks = [];

            mediaRecorder.onstart = () => {
                isRecording = true;
                button.innerHTML = uk_icon_stop;
                button.title = 'End Recording';
                // Add "Recording..." indicator to chat
                // recordingIndicatorEl = addMessage("🎙 Recording...", "user-message recording-indicator", "You", getCurrentTime());
            };

            mediaRecorder.ondataavailable = (event) => {
                if (event.data.size > 0) {
                    audioChunks.push(event.data);
                }
            };

            mediaRecorder.onstop = async () => {
                isRecording = false;

                // if (recordingIndicatorEl) {
                //     recordingIndicatorEl.querySelector('.message').textContent = "🎙 Processing...";
                // }

                if (audioChunks.length === 0) {
                    if (recordingIndicatorEl) recordingIndicatorEl.remove();
                    console.error("No Audio recorded.");
                    addErrorMessage("No audio was recorded. Please try again.");
                    return;
                }

                const audioBlob = new Blob(audioChunks, { type: "audio/wav" });

                // Process the audio to the correct format - 16000 Hz Mono
                try {
                    const resampledBlob = await processAudio(audioBlob, 16000);
                    const audioUrl = URL.createObjectURL(resampledBlob);

                    // Replace indicator with audio message
                    // if (recordingIndicatorEl) {
                    //     recordingIndicatorEl.remove(); // remove indicator bubble
                    //     recordingIndicatorEl = null;
                    // }
                    // addAudioMessage(audioUrl, "user-message", "You", getCurrentTime());

                    const reader = new FileReader();
                    reader.readAsDataURL(resampledBlob); // Use the resampled blob
                    reader.onloadend = () => {
                        const base64Audio = reader.result.split(",")[1];
                        if (base64Audio) {
                            ws_stream.send(JSON.stringify({ type: "audio", payload: base64Audio, topic_name: button.id }));
                        }
                    };
                } catch (error) {
                    console.error("Failed to process audio:", error);
                    if (recordingIndicatorEl) recordingIndicatorEl.remove();
                    // addErrorMessage("Error: Could not process recorded audio.");
                }

                button.innerHTML = uk_icon_recording;
                button.title = "Record";
            };

            mediaRecorder.start();
        } catch (error) {
            console.error("Error accessing microphone:", error);
            // addErrorMessage("Error: Could not access the microphone. Please grant permission.");
        }
    }
};

// Create a single AudioContext to be reused
const audioContext = new (window.AudioContext || window.webkitAudioContext)();

async function processAudio(audioBlob, targetSampleRate) {
    // 1. Decode the audio file into an AudioBuffer
    const arrayBuffer = await audioBlob.arrayBuffer();
    const originalAudioBuffer = await audioContext.decodeAudioData(arrayBuffer);

    const numberOfChannels = originalAudioBuffer.numberOfChannels;
    const originalSampleRate = originalAudioBuffer.sampleRate;

    // 2. If it's already in the target format, no need to process
    if (originalSampleRate === targetSampleRate && numberOfChannels === 1) {
        return audioBlob;
    }

    // 3. Resample and convert to mono using an OfflineAudioContext
    const duration = originalAudioBuffer.duration;
    const offlineContext = new OfflineAudioContext(1, duration * targetSampleRate, targetSampleRate);

    const source = offlineContext.createBufferSource();
    source.buffer = originalAudioBuffer;
    source.connect(offlineContext.destination);
    source.start(0);

    const resampledAudioBuffer = await offlineContext.startRendering();

    // 4. Encode the new AudioBuffer into a WAV file Blob
    return bufferToWav(resampledAudioBuffer);
}

function bufferToWav(buffer) {
    const numOfChan = buffer.numberOfChannels;
    const length = buffer.length * numOfChan * 2 + 44;
    const bufferArr = new ArrayBuffer(length);
    const view = new DataView(bufferArr);
    const channels = [];
    let i;
    let sample;
    let offset = 0;
    let pos = 0;

    // WAV header
    setUint32(0x46464952); // "RIFF"
    setUint32(length - 8); // file length - 8
    setUint32(0x45564157); // "WAVE"
    setUint32(0x20746d66); // "fmt " chunk
    setUint32(16); // length of fmt data
    setUint16(1); // PCM - integer samples
    setUint16(numOfChan); // channel count
    setUint32(buffer.sampleRate); // sample rate
    setUint32(buffer.sampleRate * 2 * numOfChan); // byte rate
    setUint16(numOfChan * 2); // block align
    setUint16(16); // bits per sample
    setUint32(0x61746164); // "data" - chunk
    setUint32(length - pos - 4); // chunk length

    // Write interleaved PCM data
    for (i = 0; i < buffer.numberOfChannels; i++) {
        channels.push(buffer.getChannelData(i));
    }

    while (pos < length) {
        for (i = 0; i < numOfChan; i++) {
            sample = Math.max(-1, Math.min(1, channels[i][offset])); // clamp
            sample = (0.5 + sample < 0 ? sample * 32768 : sample * 32767) | 0; // scale to 16-bit signed int
            view.setInt16(pos, sample, true); // write 16-bit sample
            pos += 2;
        }
        offset++;
    }

    return new Blob([view], { type: "audio/wav" });

    function setUint16(data) {
        view.setUint16(pos, data, true);
        pos += 2;
    }

    function setUint32(data) {
        view.setUint32(pos, data, true);
        pos += 4;
    }
}
