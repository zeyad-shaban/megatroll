// ─────────────────────────────────────────────────────────────────────────────
// ROS2 Joystick Controller
// Publishes geometry_msgs/TwistStamped on /cmd_vel
// Joystick distance from center maps to PWM -100…+100 with a quadratic curve
// ─────────────────────────────────────────────────────────────────────────────

// ─── Config ──────────────────────────────────────────────────────────────────
const ROS_URL = 'ws://zeyadcodepi.local:9090';
const PUBLISH_HZ = 20;       // how often to send commands (Hz)
const RECONNECT_MS = 2000;     // ms between reconnect attempts
const DEAD_ZONE = 0.04;     // normalized joystick dead zone radius
const STOP_DELAY = 100;      // ms debounce before sending stop on release

// Curve shaping: 1.0 = linear, 2.0 = quadratic, 3.0 = cubic
// Quadratic (2.0) gives much finer control at low speeds without losing max speed.
// Tune this alongside MOTOR_MIN on the STM32 side.
const CURVE_EXP = 2.0;

// Speed multiplier from the slider — caps max output, does NOT change the curve shape
let speedScale = 1.0;

// ─── DOM refs ────────────────────────────────────────────────────────────────
const statusDiv = document.getElementById('statusMsg');
const canvas = document.getElementById('joystickCanvas');
const speedSlider = document.getElementById('speedScale');
const linearOut = document.getElementById('linearVal');
const angularOut = document.getElementById('angularVal');

// ─── ROS2 connection ─────────────────────────────────────────────────────────
const ros = new ROSLIB.Ros({ url: ROS_URL });
let reconnectTimer = null;

function setConnected(connected) {
    if (connected) {
        clearInterval(reconnectTimer);
        reconnectTimer = null;
        statusDiv.textContent = '✅ Connected';
        statusDiv.className = 'connected';
        canvas.style.opacity = '1';
        canvas.style.pointerEvents = 'auto';
    } else {
        canvas.style.opacity = '0.45';
        canvas.style.pointerEvents = 'none';
        if (!reconnectTimer) {
            reconnectTimer = setInterval(() => ros.connect(ROS_URL), RECONNECT_MS);
        }
    }
}

ros.on('connection', () => { setConnected(true); });
ros.on('error', (err) => { statusDiv.textContent = `❌ Error: ${err}`; statusDiv.className = 'error'; setConnected(false); });
ros.on('close', () => { statusDiv.textContent = '⚠️ Disconnected – reconnecting…'; statusDiv.className = 'disconnected'; setConnected(false); });

// ─── Publisher ───────────────────────────────────────────────────────────────
const cmdVelTopic = new ROSLIB.Topic({
    ros,
    name: '/cmd_vel',
    messageType: 'geometry_msgs/TwistStamped',
});

function publishTwist(linear, angular) {
    if (!ros.isConnected) return;
    cmdVelTopic.publish(new ROSLIB.Message({
        header: { stamp: { sec: 0, nanosec: 0 }, frame_id: 'base_link' },
        twist: {
            linear: { x: linear, y: 0, z: 0 },
            angular: { x: 0, y: 0, z: angular },
        },
    }));
    if (linearOut) linearOut.textContent = linear.toFixed(3);
    if (angularOut) angularOut.textContent = angular.toFixed(3);
}

// ─── Curve shaping ───────────────────────────────────────────────────────────
// Applies dead zone then raises the normalized value to CURVE_EXP.
// This compresses the low-speed range so the first half of joystick travel
// produces finer speed steps instead of jumping straight to fast.
function shapedAxis(raw) {
    // Dead zone: ignore tiny inputs near center
    if (Math.abs(raw) < DEAD_ZONE) return 0;
    // Re-scale so output begins at 0 just outside the dead zone
    const rescaled = (raw - Math.sign(raw) * DEAD_ZONE) / (1 - DEAD_ZONE);
    // Apply power curve, preserving sign
    return Math.sign(rescaled) * Math.pow(Math.abs(rescaled), CURVE_EXP);
}

// ─── Joystick geometry ───────────────────────────────────────────────────────
const SIZE = canvas.width;
const CX = SIZE / 2;
const CY = SIZE / 2;
const MAX_R = SIZE * 0.32;
const KNOB_R = SIZE * 0.11;

let joyNormX = 0;   // raw normalized -1…+1 (right = +)
let joyNormY = 0;   // raw normalized -1…+1 (up = +)
let isActive = false;

// ─── Canvas drawing ──────────────────────────────────────────────────────────
const ctx = canvas.getContext('2d');

function drawJoystick() {
    ctx.clearRect(0, 0, SIZE, SIZE);

    ctx.beginPath();
    ctx.arc(CX, CY, MAX_R + 6, 0, Math.PI * 2);
    ctx.fillStyle = '#1a2630';
    ctx.fill();

    ctx.beginPath();
    ctx.arc(CX, CY, MAX_R, 0, Math.PI * 2);
    ctx.fillStyle = '#2c3e4a';
    ctx.fill();
    ctx.strokeStyle = '#4a6070';
    ctx.lineWidth = 1.5;
    ctx.stroke();

    ctx.setLineDash([4, 6]);
    ctx.strokeStyle = 'rgba(255,255,255,0.12)';
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(CX - MAX_R, CY); ctx.lineTo(CX + MAX_R, CY);
    ctx.moveTo(CX, CY - MAX_R); ctx.lineTo(CX, CY + MAX_R);
    ctx.stroke();
    ctx.setLineDash([]);

    // Clamp knob to circle
    let kx = CX + joyNormX * MAX_R;
    let ky = CY - joyNormY * MAX_R;
    const d = Math.hypot(kx - CX, ky - CY);
    if (d > MAX_R) { kx = CX + (kx - CX) / d * MAX_R; ky = CY + (ky - CY) / d * MAX_R; }

    if (isActive) { ctx.shadowColor = '#e67e22aa'; ctx.shadowBlur = 18; }
    ctx.beginPath(); ctx.arc(kx, ky, KNOB_R, 0, Math.PI * 2); ctx.fillStyle = '#c0622a'; ctx.fill();
    ctx.beginPath(); ctx.arc(kx, ky, KNOB_R * 0.78, 0, Math.PI * 2); ctx.fillStyle = '#e67e22'; ctx.fill();
    ctx.shadowBlur = 0;

    ctx.beginPath(); ctx.arc(CX, CY, 3, 0, Math.PI * 2); ctx.fillStyle = 'rgba(255,255,255,0.2)'; ctx.fill();
}

// ─── Input → normalized coords ───────────────────────────────────────────────
function toNormalized(clientX, clientY) {
    const rect = canvas.getBoundingClientRect();
    let dx = (clientX - rect.left) * (SIZE / rect.width) - CX;
    let dy = (clientY - rect.top) * (SIZE / rect.height) - CY;
    const dist = Math.hypot(dx, dy);
    if (dist > MAX_R) { dx = dx / dist * MAX_R; dy = dy / dist * MAX_R; }
    return { x: dx / MAX_R, y: -dy / MAX_R };
}

// ─── Publish loop ────────────────────────────────────────────────────────────
let publishTimer = null;
let stopTimer = null;

function startPublishing() {
    if (publishTimer) return;
    publishTimer = setInterval(() => {
        if (!isActive) return;
        // Apply curve shaping to each axis independently
        const vx = shapedAxis(joyNormX) * speedScale;
        const vy = shapedAxis(joyNormY) * speedScale;
        publishTwist(vy, -vx);  // linear = forward/back, angular = left/right (ROS: left = +)
    }, 1000 / PUBLISH_HZ);
}

function stopPublishing() {
    if (publishTimer) { clearInterval(publishTimer); publishTimer = null; }
}

// ─── Event handlers ──────────────────────────────────────────────────────────
function onStart(e) {
    if (!ros.isConnected) return;
    e.preventDefault();
    clearTimeout(stopTimer); stopTimer = null;
    isActive = true;
    const pt = e.touches ? e.touches[0] : e;
    ({ x: joyNormX, y: joyNormY } = toNormalized(pt.clientX, pt.clientY));
    drawJoystick();
    startPublishing();
}

function onMove(e) {
    if (!isActive) return;
    e.preventDefault();
    const pt = e.touches ? e.touches[0] : e;
    ({ x: joyNormX, y: joyNormY } = toNormalized(pt.clientX, pt.clientY));
    drawJoystick();
}

function onEnd(e) {
    if (!isActive) return;
    e.preventDefault();
    isActive = false; joyNormX = 0; joyNormY = 0;
    drawJoystick();
    stopPublishing();
    stopTimer = setTimeout(() => { publishTwist(0, 0); stopTimer = null; }, STOP_DELAY);
}

canvas.addEventListener('mousedown', onStart);
window.addEventListener('mousemove', onMove);
window.addEventListener('mouseup', onEnd);
canvas.addEventListener('touchstart', onStart, { passive: false });
window.addEventListener('touchmove', onMove, { passive: false });
window.addEventListener('touchend', onEnd, { passive: false });

speedSlider.addEventListener('input', e => { speedScale = parseFloat(e.target.value); });

// ─── Init ─────────────────────────────────────────────────────────────────────
setConnected(false);
drawJoystick();
