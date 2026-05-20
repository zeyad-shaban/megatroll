// ROS 2 browser joystick for a simple differential-drive car.
// The browser publishes normal cmd_vel values:
//   linear.x  = forward/back command
//   angular.z = left/right turn command

const ROS_URL = 'ws://zeyadcodepi.local:9090';
const PUBLISH_HZ = 20;
const RECONNECT_MS = 2000;
const ZERO_BURST_COUNT = 4;
const DEAD_ZONE = 0.06;
const EXPO = 1.7;
const MIN_PWM_PERCENT = 30;
const WHEEL_DEAD_ZONE = 0.02;

const statusDiv = document.getElementById('statusMsg');
const canvas = document.getElementById('joystickCanvas');
const speedSlider = document.getElementById('speedScale');
const speedValue = document.getElementById('speedValue');
const linearOut = document.getElementById('linearVal');
const angularOut = document.getElementById('angularVal');
const leftOut = document.getElementById('leftVal');
const rightOut = document.getElementById('rightVal');

let speedScale = Number(speedSlider.value);
let reconnectTimer = null;
let publishTimer = null;
let activePointerId = null;
let zeroBurstRemaining = ZERO_BURST_COUNT;
let joyX = 0;
let joyY = 0;

const ros = new ROSLIB.Ros({ url: ROS_URL });
const cmdVelTopic = new ROSLIB.Topic({
    ros,
    name: '/cmd_vel',
    messageType: 'geometry_msgs/TwistStamped',
});

const ctx = canvas.getContext('2d');
const view = {
    size: 0,
    cx: 0,
    cy: 0,
    maxR: 0,
    knobR: 0,
};

function clamp(value, min, max) {
    return Math.max(min, Math.min(max, value));
}

function shapeAxis(value) {
    const magnitude = Math.abs(value);
    if (magnitude < DEAD_ZONE) return 0;

    const normalized = (magnitude - DEAD_ZONE) / (1 - DEAD_ZONE);
    return Math.sign(value) * Math.pow(normalized, EXPO);
}

function getCommand() {
    const linear = shapeAxis(joyY) * speedScale;
    const angular = shapeAxis(joyX) * speedScale;
    return {
        linear: clamp(linear, -1, 1),
        angular: clamp(angular, -1, 1),
    };
}

function previewWheelMix(linear, angular) {
    let left = linear - angular;
    let right = linear + angular;
    const maxMag = Math.max(1, Math.abs(left), Math.abs(right));
    return {
        left: scaleWheelForDisplay(left / maxMag),
        right: scaleWheelForDisplay(right / maxMag),
    };
}

function scaleWheelForDisplay(value) {
    const magnitude = Math.abs(value);
    if (magnitude < WHEEL_DEAD_ZONE) return 0;

    const pwm = MIN_PWM_PERCENT + magnitude * (100 - MIN_PWM_PERCENT);
    return Math.sign(value) * pwm;
}

function setStatus(kind, text) {
    statusDiv.className = `status ${kind}`;
    statusDiv.textContent = text;
}

function setConnected(ok) {
    canvas.classList.toggle('is-disabled', !ok);

    if (ok) {
        if (reconnectTimer) {
            clearInterval(reconnectTimer);
            reconnectTimer = null;
        }
        setStatus('connected', 'Connected');
        startPublishing();
        return;
    }

    activePointerId = null;
    joyX = 0;
    joyY = 0;
    drawJoystick();
    stopPublishing();

    if (!reconnectTimer) {
        reconnectTimer = setInterval(() => {
            if (!ros.isConnected) ros.connect(ROS_URL);
        }, RECONNECT_MS);
    }
}

ros.on('connection', () => setConnected(true));
ros.on('error', (err) => {
    setStatus('error', `Connection error: ${err?.message || err || 'unknown'}`);
    setConnected(false);
});
ros.on('close', () => {
    setStatus('disconnected', 'Disconnected. Reconnecting...');
    setConnected(false);
});

function publishCmd(linear, angular) {
    if (!ros.isConnected) return;

    cmdVelTopic.publish(new ROSLIB.Message({
        header: { stamp: { sec: 0, nanosec: 0 }, frame_id: 'base_link' },
        twist: {
            linear: { x: linear, y: 0, z: 0 },
            angular: { x: 0, y: 0, z: angular },
        },
    }));
}

function updateReadout(linear, angular) {
    const { left, right } = previewWheelMix(linear, angular);
    linearOut.textContent = linear.toFixed(2);
    angularOut.textContent = angular.toFixed(2);
    leftOut.textContent = `${Math.round(left)}%`;
    rightOut.textContent = `${Math.round(right)}%`;
}

function publishCurrentCommand() {
    const isActive = activePointerId !== null;
    const { linear, angular } = isActive ? getCommand() : { linear: 0, angular: 0 };

    if (!isActive && zeroBurstRemaining <= 0) {
        updateReadout(0, 0);
        return;
    }

    publishCmd(linear, angular);
    updateReadout(linear, angular);

    if (!isActive) zeroBurstRemaining -= 1;
}

function startPublishing() {
    if (publishTimer) return;
    publishTimer = setInterval(publishCurrentCommand, 1000 / PUBLISH_HZ);
    publishCurrentCommand();
}

function stopPublishing() {
    if (!publishTimer) return;
    clearInterval(publishTimer);
    publishTimer = null;
    updateReadout(0, 0);
}

function resizeCanvas() {
    const rect = canvas.getBoundingClientRect();
    const dpr = window.devicePixelRatio || 1;

    canvas.width = Math.round(rect.width * dpr);
    canvas.height = Math.round(rect.height * dpr);
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);

    view.size = Math.min(rect.width, rect.height);
    view.cx = rect.width / 2;
    view.cy = rect.height / 2;
    view.maxR = view.size * 0.34;
    view.knobR = view.size * 0.12;

    drawJoystick();
}

function drawRing(radius, color, width = 1) {
    ctx.beginPath();
    ctx.arc(view.cx, view.cy, radius, 0, Math.PI * 2);
    ctx.strokeStyle = color;
    ctx.lineWidth = width;
    ctx.stroke();
}

function drawJoystick() {
    const rect = canvas.getBoundingClientRect();
    ctx.clearRect(0, 0, rect.width, rect.height);

    const gradient = ctx.createRadialGradient(
        view.cx,
        view.cy,
        view.knobR,
        view.cx,
        view.cy,
        view.maxR + 42,
    );
    gradient.addColorStop(0, '#28394a');
    gradient.addColorStop(1, '#101820');

    ctx.beginPath();
    ctx.arc(view.cx, view.cy, view.maxR + 42, 0, Math.PI * 2);
    ctx.fillStyle = gradient;
    ctx.fill();

    drawRing(view.maxR, 'rgba(148, 163, 184, 0.38)', 2);
    drawRing(view.maxR * 0.55, 'rgba(148, 163, 184, 0.18)', 1);

    ctx.strokeStyle = 'rgba(148, 163, 184, 0.28)';
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(view.cx - view.maxR, view.cy);
    ctx.lineTo(view.cx + view.maxR, view.cy);
    ctx.moveTo(view.cx, view.cy - view.maxR);
    ctx.lineTo(view.cx, view.cy + view.maxR);
    ctx.stroke();

    const knobX = view.cx + joyX * view.maxR;
    const knobY = view.cy - joyY * view.maxR;
    const isActive = activePointerId !== null;

    if (isActive) {
        ctx.beginPath();
        ctx.moveTo(view.cx, view.cy);
        ctx.lineTo(knobX, knobY);
        ctx.strokeStyle = 'rgba(45, 212, 191, 0.55)';
        ctx.lineWidth = 4;
        ctx.lineCap = 'round';
        ctx.stroke();
    }

    ctx.shadowColor = isActive ? 'rgba(45, 212, 191, 0.45)' : 'rgba(0, 0, 0, 0.35)';
    ctx.shadowBlur = isActive ? 24 : 14;
    ctx.beginPath();
    ctx.arc(knobX, knobY, view.knobR, 0, Math.PI * 2);
    ctx.fillStyle = isActive ? '#2dd4bf' : '#e2e8f0';
    ctx.fill();
    ctx.shadowBlur = 0;

    ctx.beginPath();
    ctx.arc(knobX, knobY, view.knobR * 0.42, 0, Math.PI * 2);
    ctx.fillStyle = isActive ? '#0f766e' : '#64748b';
    ctx.fill();
}

function pointerToJoystick(event) {
    const rect = canvas.getBoundingClientRect();
    let dx = event.clientX - rect.left - view.cx;
    let dy = event.clientY - rect.top - view.cy;
    const distance = Math.hypot(dx, dy);

    if (distance > view.maxR) {
        dx = (dx / distance) * view.maxR;
        dy = (dy / distance) * view.maxR;
    }

    return {
        x: dx / view.maxR,
        y: -dy / view.maxR,
    };
}

function releaseJoystick() {
    activePointerId = null;
    joyX = 0;
    joyY = 0;
    zeroBurstRemaining = ZERO_BURST_COUNT;
    publishCmd(0, 0);
    updateReadout(0, 0);
    drawJoystick();
}

canvas.addEventListener('pointerdown', (event) => {
    if (!ros.isConnected) return;

    event.preventDefault();
    activePointerId = event.pointerId;
    canvas.setPointerCapture(event.pointerId);
    zeroBurstRemaining = ZERO_BURST_COUNT;
    ({ x: joyX, y: joyY } = pointerToJoystick(event));
    drawJoystick();
    publishCurrentCommand();
});

canvas.addEventListener('pointermove', (event) => {
    if (event.pointerId !== activePointerId) return;

    event.preventDefault();
    ({ x: joyX, y: joyY } = pointerToJoystick(event));
    drawJoystick();
});

canvas.addEventListener('pointerup', (event) => {
    if (event.pointerId !== activePointerId) return;
    event.preventDefault();
    releaseJoystick();
});

canvas.addEventListener('pointercancel', (event) => {
    if (event.pointerId !== activePointerId) return;
    releaseJoystick();
});

canvas.addEventListener('lostpointercapture', () => {
    if (activePointerId !== null) releaseJoystick();
});

speedSlider.addEventListener('input', (event) => {
    speedScale = Number(event.target.value);
    speedValue.textContent = `${Math.round(speedScale * 100)}%`;
});

window.addEventListener('resize', resizeCanvas);
window.addEventListener('blur', releaseJoystick);

setConnected(false);
speedValue.textContent = `${Math.round(speedScale * 100)}%`;
resizeCanvas();
