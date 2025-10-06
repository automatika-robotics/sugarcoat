const videoFrame = document.getElementsByName('video-frame');

// Establish WebSocket connection (protocol aware)
const wsProtocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
const ws = new WebSocket(`${wsProtocol}//${window.location.host}/ws_stream`);

let currentStreamingMessageElement = null;
let currentStreamingTimestamp = null;

ws.onopen = () => {
    console.log("WebSocket connection established");
};

ws.onmessage = (event) => {
    const data = JSON.parse(event.data);

    // only images received here
    const target = Array.from(videoFrame).find(img => img.id === data.topic);
    target.src = 'data:image/jpeg;base64,' + data.payload;
};

ws.onerror = (error) => {
    console.error("WebSocket error: ", error);
};

ws.onclose = () => {
    console.log("WebSocket connection closed");
};


