// ---------- ROS2 setup ----------
const URL_REAL = "ws://zeyadcodepi.local:9090";
const URL_GZ = "ws://localhost:9090";

const ENDPOINTS = {
    gz: { label: "GZ", url: URL_GZ },
    real: { label: "Real", url: URL_REAL },
};

const ros = new ROSLIB.Ros();
const RECONNECT_DELAY = 2000;
const AUTO_ENDPOINT_ORDER = ["gz", "real"];

let connectionMode = "auto";
let activeEndpoint = null;
let connectedEndpoint = null;
let reconnectTimer = null;
let autoEndpointIndex = 0;

const statusDiv = document.getElementById("statusMsg");
const connectedEndpointLabel = document.getElementById("connectedEndpointLabel");
const modeButtons = [...document.querySelectorAll(".mode-btn")];
const canvas = document.getElementById("joystickCanvas");
const speedSlider = document.getElementById("speedScale");
const speedScaleValue = document.getElementById("speedScaleValue");
const cameraImg = document.getElementById("cameraImage");
const cameraPlaceholder = document.getElementById("cameraPlaceholder");
const cameraStatusDiv = document.getElementById("cameraStatus");
const fpsVal = document.getElementById("fpsVal");
const autoEnabledToggle = document.getElementById("autoEnabledToggle");
const autoStateText = document.getElementById("autoStateText");
const joystickLockBadge = document.getElementById("joystickLockBadge");
const eventLog = document.getElementById("eventLog");

let lastFrameTime = null;
let fps = 0;
let autoEnabled = false;

function addEventLog(message) {
    const row = document.createElement("div");
    row.textContent = `${new Date().toLocaleTimeString()}  ${message}`;
    eventLog.prepend(row);

    while (eventLog.children.length > 8) {
        eventLog.removeChild(eventLog.lastChild);
    }
}

function setStatus(text, kind = "warn") {
    statusDiv.textContent = text;
    statusDiv.className = `status-pill status-${kind}`;
}

function updateConnectionUi() {
    modeButtons.forEach((button) => {
        const mode = button.dataset.mode;
        button.classList.toggle("is-selected", mode === connectionMode);
        button.classList.toggle("is-connected", mode === connectedEndpoint);
    });

    if (connectedEndpoint) {
        connectedEndpointLabel.textContent = `${ENDPOINTS[connectedEndpoint].label} connected`;
        connectedEndpointLabel.className =
            "rounded-full bg-emerald-500 px-2 py-1 text-xs font-semibold text-emerald-950";
    } else if (activeEndpoint) {
        connectedEndpointLabel.textContent = `Trying ${ENDPOINTS[activeEndpoint].label}`;
        connectedEndpointLabel.className =
            "rounded-full bg-amber-500/15 px-2 py-1 text-xs font-semibold text-amber-200";
    } else {
        connectedEndpointLabel.textContent = "Offline";
        connectedEndpointLabel.className =
            "rounded-full bg-zinc-800 px-2 py-1 text-xs font-semibold text-zinc-400";
    }
}

function getNextEndpoint() {
    if (connectionMode === "gz" || connectionMode === "real") {
        return connectionMode;
    }

    const endpoint = AUTO_ENDPOINT_ORDER[autoEndpointIndex % AUTO_ENDPOINT_ORDER.length];
    autoEndpointIndex += 1;
    return endpoint;
}

function clearReconnectTimer() {
    if (reconnectTimer) {
        clearTimeout(reconnectTimer);
        reconnectTimer = null;
    }
}

function scheduleReconnect() {
    if (reconnectTimer || ros.isConnected) return;

    reconnectTimer = setTimeout(() => {
        reconnectTimer = null;
        connectToSelectedMode();
    }, RECONNECT_DELAY);
}

function connectToSelectedMode() {
    clearReconnectTimer();
    activeEndpoint = getNextEndpoint();
    updateConnectionUi();
    setStatus(`Connecting to ${ENDPOINTS[activeEndpoint].label}`, "warn");

    try {
        ros.connect(ENDPOINTS[activeEndpoint].url);
    } catch (error) {
        addEventLog(`Connect failed: ${error.message || error}`);
        scheduleReconnect();
    }
}

function changeConnectionMode(nextMode) {
    if (connectionMode === nextMode && (ros.isConnected || reconnectTimer)) return;

    connectionMode = nextMode;
    autoEndpointIndex = 0;
    clearReconnectTimer();
    connectedEndpoint = null;
    activeEndpoint = null;
    updateConnectionUi();
    addEventLog(`Connection mode set to ${nextMode.toUpperCase()}`);

    if (ros.isConnected) {
        ros.close();
    } else {
        connectToSelectedMode();
    }
}

modeButtons.forEach((button) => {
    button.addEventListener("click", () => changeConnectionMode(button.dataset.mode));
});

ros.on("connection", () => {
    clearReconnectTimer();
    connectedEndpoint = activeEndpoint;
    setStatus(`ROS2 connected via ${ENDPOINTS[connectedEndpoint].label}`, "ok");
    addEventLog(`Connected to ${ENDPOINTS[connectedEndpoint].label}`);
    updateConnectionUi();
    publishAutoEnabled(autoEnabled);
    updateManualControlsState();
    cameraStatusDiv.textContent = "Waiting for camera frames";
});

ros.on("error", (err) => {
    const endpointLabel = activeEndpoint ? ENDPOINTS[activeEndpoint].label : "ROS2";
    connectedEndpoint = null;
    stopJoystick({ sendStop: false });
    setStatus(`${endpointLabel} unavailable`, "error");
    cameraStatusDiv.textContent = "Camera unavailable";
    addEventLog(`Connection error on ${endpointLabel}`);
    updateConnectionUi();
    updateManualControlsState();
    scheduleReconnect();
});

ros.on("close", () => {
    connectedEndpoint = null;
    stopJoystick({ sendStop: false });
    setStatus("Disconnected, reconnecting", "warn");
    cameraStatusDiv.textContent = "Camera disconnected";
    updateConnectionUi();
    updateManualControlsState();
    scheduleReconnect();
});

// ---------- Topics ----------
const cmdVelPub = new ROSLIB.Topic({
    ros,
    name: "/cmd_vel",
    messageType: "geometry_msgs/msg/TwistStamped",
});

const cmdGripPub = new ROSLIB.Topic({
    ros,
    name: "/gripper_controller/commands",
    messageType: "std_msgs/msg/Float64MultiArray",
});

const autoEnabledPub = new ROSLIB.Topic({
    ros,
    name: "/auto_enabled",
    messageType: "std_msgs/msg/Bool",
});

const cameraTopic = new ROSLIB.Topic({
    ros,
    name: "/camera/image/compressed",
    messageType: "sensor_msgs/msg/CompressedImage",
});

const targetStateTopic = new ROSLIB.Topic({
    ros,
    name: "/target_state",
    messageType: "megajaw_interfaces/msg/TargetControl",
});

// ---------- Camera feed subscription ----------
cameraTopic.subscribe((message) => {
    cameraImg.src = `data:image/jpeg;base64,${message.data}`;
    cameraPlaceholder.classList.add("hidden");
    cameraStatusDiv.textContent = "Live";

    const now = Date.now();
    if (lastFrameTime) {
        const dt = (now - lastFrameTime) / 1000;
        fps = 1 / dt;
        fpsVal.textContent = fps.toFixed(1);
    }
    lastFrameTime = now;
});

// ---------- Gripper setup ----------
const gripperStatusDiv = document.getElementById("gripperStatus");
const btnOpen = document.getElementById("btnOpen");
const btnClose = document.getElementById("btnClose");

function setGripperStatus(text, className) {
    gripperStatusDiv.textContent = text;
    gripperStatusDiv.className = className;
}

btnOpen.addEventListener("click", () => {
    if (!ros.isConnected || autoEnabled) return;

    const msg = new ROSLIB.Message({ data: [1.0, -1.0] });
    cmdGripPub.publish(msg);
    setGripperStatus(
        "Open",
        "rounded-full bg-emerald-500/15 px-2 py-1 text-xs font-semibold text-emerald-300",
    );
    addEventLog("Gripper open command sent");
});

btnClose.addEventListener("click", () => {
    if (!ros.isConnected || autoEnabled) return;

    const msg = new ROSLIB.Message({ data: [0.0, 0.0] });
    cmdGripPub.publish(msg);
    setGripperStatus(
        "Closed",
        "rounded-full bg-rose-500/15 px-2 py-1 text-xs font-semibold text-rose-300",
    );
    addEventLog("Gripper close command sent");
});

// ---------- Joystick parameters ----------
const size = 250;
const centerX = size / 2;
const centerY = size / 2;
const maxRadius = 80;

let active = false;
let joyX = 0;
let joyY = 0;
let animationId = null;
let speedScale = parseFloat(speedSlider.value);

speedSlider.addEventListener("input", (e) => {
    speedScale = parseFloat(e.target.value);
    speedScaleValue.textContent = Math.round(speedScale * 100);
});

const ctx = canvas.getContext("2d");

function draw() {
    ctx.clearRect(0, 0, size, size);

    const gradient = ctx.createRadialGradient(centerX, centerY, 22, centerX, centerY, maxRadius + 20);
    gradient.addColorStop(0, "#27272a");
    gradient.addColorStop(1, "#09090b");

    ctx.beginPath();
    ctx.arc(centerX, centerY, maxRadius + 14, 0, 2 * Math.PI);
    ctx.fillStyle = gradient;
    ctx.fill();
    ctx.strokeStyle = "rgba(255,255,255,0.10)";
    ctx.lineWidth = 2;
    ctx.stroke();

    ctx.beginPath();
    ctx.arc(centerX, centerY, maxRadius, 0, 2 * Math.PI);
    ctx.strokeStyle = "rgba(34,211,238,0.55)";
    ctx.lineWidth = 2;
    ctx.stroke();

    ctx.beginPath();
    ctx.moveTo(centerX - maxRadius, centerY);
    ctx.lineTo(centerX + maxRadius, centerY);
    ctx.moveTo(centerX, centerY - maxRadius);
    ctx.lineTo(centerX, centerY + maxRadius);
    ctx.strokeStyle = "rgba(255,255,255,0.16)";
    ctx.lineWidth = 1;
    ctx.stroke();

    let knobX = centerX + joyX * maxRadius;
    let knobY = centerY - joyY * maxRadius;
    const dx = knobX - centerX;
    const dy = knobY - centerY;
    const dist = Math.hypot(dx, dy);

    if (dist > maxRadius) {
        knobX = centerX + (dx / dist) * maxRadius;
        knobY = centerY + (dy / dist) * maxRadius;
    }

    ctx.beginPath();
    ctx.arc(knobX, knobY, 28, 0, 2 * Math.PI);
    ctx.fillStyle = "#22d3ee";
    ctx.fill();
    ctx.strokeStyle = "rgba(255,255,255,0.72)";
    ctx.lineWidth = 3;
    ctx.stroke();

    ctx.beginPath();
    ctx.arc(knobX - 8, knobY - 8, 6, 0, 2 * Math.PI);
    ctx.fillStyle = "rgba(255,255,255,0.55)";
    ctx.fill();
}

function getNormalizedCoords(clientX, clientY) {
    const rect = canvas.getBoundingClientRect();
    const scaleX = canvas.width / rect.width;
    const scaleY = canvas.height / rect.height;
    let canvasX = (clientX - rect.left) * scaleX;
    let canvasY = (clientY - rect.top) * scaleY;

    canvasX = Math.min(Math.max(canvasX, 0), size);
    canvasY = Math.min(Math.max(canvasY, 0), size);

    let dx = canvasX - centerX;
    let dy = canvasY - centerY;
    const dist = Math.hypot(dx, dy);

    if (dist > maxRadius) {
        dx = (dx / dist) * maxRadius;
        dy = (dy / dist) * maxRadius;
    }

    return {
        x: dx / maxRadius,
        y: -dy / maxRadius,
    };
}

function canUseManualControls() {
    return ros.isConnected && !autoEnabled;
}

function stopJoystick({ sendStop = true } = {}) {
    active = false;

    if (animationId) {
        cancelAnimationFrame(animationId);
        animationId = null;
    }

    joyX = 0;
    joyY = 0;
    draw();

    if (sendStop) {
        publishTwist(0, 0, { force: true });
    }
}

function updateManualControlsState() {
    const locked = !canUseManualControls();
    canvas.classList.toggle("is-locked", locked);
    speedSlider.disabled = locked;
    btnOpen.disabled = locked;
    btnClose.disabled = locked;

    if (autoEnabled) {
        joystickLockBadge.textContent = "Auto lock";
        joystickLockBadge.className =
            "rounded-full bg-emerald-500/15 px-2 py-1 text-xs font-semibold text-emerald-300";
    } else if (!ros.isConnected) {
        joystickLockBadge.textContent = "Offline";
        joystickLockBadge.className =
            "rounded-full bg-zinc-800 px-2 py-1 text-xs font-semibold text-zinc-400";
    } else {
        joystickLockBadge.textContent = "Manual";
        joystickLockBadge.className =
            "rounded-full bg-cyan-500/15 px-2 py-1 text-xs font-semibold text-cyan-300";
    }
}

function handleStart(e) {
    if (!canUseManualControls()) return;

    e.preventDefault();
    active = true;

    const point = e.touches ? e.touches[0] : e;
    const { x, y } = getNormalizedCoords(point.clientX, point.clientY);
    joyX = x;
    joyY = y;
    draw();
    startPublishing();
}

function handleMove(e) {
    if (!active || !canUseManualControls()) return;

    e.preventDefault();
    const point = e.touches ? e.touches[0] : e;
    const { x, y } = getNormalizedCoords(point.clientX, point.clientY);
    joyX = x;
    joyY = y;
    draw();
}

function handleEnd(e) {
    if (!active) return;

    e.preventDefault();
    stopJoystick();
}

function startPublishing() {
    if (animationId) cancelAnimationFrame(animationId);

    function publishLoop() {
        if (active && canUseManualControls()) {
            const linear = joyY * speedScale;
            const angular = joyX * speedScale * -Math.sign(linear || 1);
            publishTwist(linear, angular);
            animationId = requestAnimationFrame(publishLoop);
        } else {
            animationId = null;
        }
    }

    publishLoop();
}

function publishTwist(linear, angular, options = {}) {
    if (!ros.isConnected || (autoEnabled && !options.force)) return;

    const twist = new ROSLIB.Message({
        twist: {
            linear: { x: linear, y: 0, z: 0 },
            angular: { x: 0, y: 0, z: angular },
        },
    });

    cmdVelPub.publish(twist);
    document.getElementById("linearVal").textContent = linear.toFixed(3);
    document.getElementById("angularVal").textContent = angular.toFixed(3);
}

// ---------- Target State Listener ----------
targetStateTopic.subscribe((msg) => {
    if (msg.target_detected) {
        document.getElementById("targetErrX").textContent = msg.err_x.toFixed(3);
        document.getElementById("targetDepth").textContent = msg.depth.toFixed(3);
    } else {
        document.getElementById("targetErrX").textContent = "NA";
        document.getElementById("targetDepth").textContent = "NA";
    }
});

// ---------- Autonomous Mode ----------
function publishAutoEnabled(enabled) {
    if (!ros.isConnected) return;

    autoEnabledPub.publish(new ROSLIB.Message({ data: enabled }));
    addEventLog(`/auto_enabled => ${enabled ? "True" : "False"}`);
}

function setAutoEnabled(enabled) {
    autoEnabled = enabled;
    autoEnabledToggle.checked = enabled;
    autoStateText.textContent = enabled ? "Autonomous drive" : "Manual control";
    autoStateText.className = enabled
        ? "text-sm font-semibold text-emerald-300"
        : "text-sm font-semibold text-zinc-300";

    stopJoystick({ sendStop: enabled });
    updateManualControlsState();
    publishAutoEnabled(enabled);
}

autoEnabledToggle.addEventListener("change", (event) => {
    setAutoEnabled(event.target.checked);
});

// ---------- Attach events (mouse + touch) ----------
canvas.addEventListener("mousedown", handleStart);
window.addEventListener("mousemove", handleMove);
window.addEventListener("mouseup", handleEnd);

canvas.addEventListener("touchstart", handleStart, { passive: false });
window.addEventListener("touchmove", handleMove, { passive: false });
window.addEventListener("touchend", handleEnd, { passive: false });
window.addEventListener("touchcancel", handleEnd, { passive: false });

draw();
updateConnectionUi();
updateManualControlsState();
connectToSelectedMode();
