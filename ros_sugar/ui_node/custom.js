const videoFrame = document.getElementsByName('video-frame');

// Establish WebSocket connection (protocol aware)
const wsProtocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
const ws_stream = new WebSocket(`${wsProtocol}//${window.location.host}/ws_stream`);


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
        if (fixedH){
            el.style.height = computedStyle.height;
        }
        else{
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
        if (e.target.closest(".fix-size")) fixH=true;

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

ws_stream.onopen = () => {
    console.log("WebSocket connection established");
};

// Display image msgs
ws_stream.onmessage = (event) => {
    const data = JSON.parse(event.data);

    // only images received here
    const target = Array.from(videoFrame).find(img => img.id === data.topic);
    if (target) {
        target.src = 'data:image/jpeg;base64,' + data.payload;
    }
};

ws_stream.onerror = (error) => {
    console.error("WebSocket error: ", error);
};

ws_stream.onclose = () => {
    console.log("WebSocket connection closed");
};

// Audio Recording Logic
let mediaRecorder;
let audioChunks = [];
let isRecording = false;
let recordingIndicatorEl = null; // reference to the indicator message in DOM

function presistForm(form){
    FormPersistence.persist(form);
}

async function startAudioRecording(button) {
    console.log("In audio recordin")
    button.classList.toggle("recording");

    if (isRecording) {
        mediaRecorder.stop();
        button.innerHTML = `<i class="fa fa-microphone"></i>`;
    } else {
        try {
            const stream = await navigator.mediaDevices.getUserMedia({ audio: true });
            mediaRecorder = new MediaRecorder(stream);
            audioChunks = [];

            mediaRecorder.onstart = () => {
                isRecording = true;
                button.innerHTML = `<i class="fa fa-stop"></i>`;
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
                    // button.innerHTML = `<i class="fa fa-microphone"></i> <span class="record-tooltip">Record</span>`;
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
                            ws_stream.send(JSON.stringify({ type: "audio", payload: base64Audio, topic_name: button.id}));
                        }
                    };
                } catch (error) {
                    console.error("Failed to process audio:", error);
                    if (recordingIndicatorEl) recordingIndicatorEl.remove();
                    // addErrorMessage("Error: Could not process recorded audio.");
                }

                button.innerHTML = `<i class="fa fa-microphone"></i>`;
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
