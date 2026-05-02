// ---------- ROS2 setup ----------
const URL = 'ws://zeyadcodepi.local:9090' // ws://localhost:9090
const ros = new ROSLIB.Ros({ url: URL});
const statusDiv = document.getElementById('statusMsg');
const canvas = document.getElementById('joystickCanvas');
const speedSlider = document.getElementById('speedScale');

let reconnectInterval = null;
const RECONNECT_DELAY = 2000; // 2 seconds

function attemptReconnect() {
    if (!ros.isConnected && !reconnectInterval) {
        reconnectInterval = setInterval(() => {
            console.log('Attempting to reconnect to ROS2...');
            ros.connect(URL);
        }, RECONNECT_DELAY);
    }
}

function stopReconnectAttempts() {
    if (reconnectInterval) {
        clearInterval(reconnectInterval);
        reconnectInterval = null;
    }
}

ros.on('connection', () => {
    stopReconnectAttempts();
    statusDiv.innerHTML = '✅ ROS2 connected – joystick active';
    statusDiv.classList.remove('error', 'disconnected');
    canvas.style.opacity = '1';
    canvas.style.pointerEvents = 'auto';
});

ros.on('error', (err) => {
    statusDiv.innerHTML = `❌ Error: ${err}`;
    statusDiv.classList.add('error');
    canvas.style.opacity = '0.5';
    canvas.style.pointerEvents = 'none';
    attemptReconnect();
});

ros.on('close', () => {
    statusDiv.innerHTML = '⚠️ Disconnected – reconnecting...';
    statusDiv.classList.add('disconnected');
    canvas.style.opacity = '0.5';
    canvas.style.pointerEvents = 'none';
    attemptReconnect();
});

const cmdVelPub = new ROSLIB.Topic({
    ros: ros,
    name: '/cmd_vel',
    messageType: 'geometry_msgs/TwistStamped'
});

// ---------- Joystick parameters ----------
const size = 250;
const centerX = size / 2, centerY = size / 2;
const maxRadius = 80;

let active = false;
let joyX = 0, joyY = 0;
let animationId = null;
let coastingTimeout = null;
const COAST_DURATION = 5000; // 5 seconds of coasting after release

let speedScale = parseFloat(speedSlider.value);

speedSlider.addEventListener('input', (e) => {
    speedScale = parseFloat(e.target.value);
});

const ctx = canvas.getContext('2d');

// ---------- Draw joystick (knob & base) ----------
function draw() {
    ctx.clearRect(0, 0, size, size);
    // outer ring
    ctx.beginPath();
    ctx.arc(centerX, centerY, maxRadius + 8, 0, 2 * Math.PI);
    ctx.fillStyle = '#1e2a32';
    ctx.fill();
    ctx.shadowBlur = 0;
    ctx.beginPath();
    ctx.arc(centerX, centerY, maxRadius, 0, 2 * Math.PI);
    ctx.fillStyle = '#3a4c5c';
    ctx.fill();
    ctx.strokeStyle = '#5d7a8c';
    ctx.lineWidth = 2;
    ctx.stroke();

    // crosshair lines
    ctx.beginPath();
    ctx.moveTo(centerX - maxRadius, centerY);
    ctx.lineTo(centerX + maxRadius, centerY);
    ctx.moveTo(centerX, centerY - maxRadius);
    ctx.lineTo(centerX, centerY + maxRadius);
    ctx.strokeStyle = '#ffffff30';
    ctx.stroke();

    // knob position
    let knobX = centerX + joyX * maxRadius;
    let knobY = centerY - joyY * maxRadius;
    // limit to circle
    const dx = knobX - centerX, dy = knobY - centerY;
    const dist = Math.hypot(dx, dy);
    if (dist > maxRadius) {
        knobX = centerX + (dx / dist) * maxRadius;
        knobY = centerY + (dy / dist) * maxRadius;
    }
    ctx.beginPath();
    ctx.arc(knobX, knobY, 28, 0, 2 * Math.PI);
    ctx.fillStyle = '#e67e22';
    ctx.fill();
    ctx.shadowBlur = 3;
    ctx.beginPath();
    ctx.arc(knobX, knobY, 22, 0, 2 * Math.PI);
    ctx.fillStyle = '#f39c12';
    ctx.fill();
    ctx.shadowBlur = 0;
}

// ---------- Convert mouse/touch position to joystick normalized values ----------
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
    let normX = dx / maxRadius;
    let normY = dy / maxRadius;
    normY = -normY;
    return { x: normX, y: normY };
}

function handleStart(e) {
    if (!ros.isConnected) return;
    e.preventDefault();
    active = true;

    // Cancel any coasting timeout
    if (coastingTimeout) {
        clearTimeout(coastingTimeout);
        coastingTimeout = null;
    }

    const point = e.touches ? e.touches[0] : e;
    const { x, y } = getNormalizedCoords(point.clientX, point.clientY);
    joyX = x;
    joyY = y;
    draw();
    startPublishing();
}

function handleMove(e) {
    if (!active) return;
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
    active = false;

    if (animationId) {
        cancelAnimationFrame(animationId);
        animationId = null;
    }

    // Immediately reset joystick visual position to center
    joyX = 0;
    joyY = 0;
    draw();

    // Start coasting: send zero velocity for 5 seconds
    publishTwist(0, 0);

    // After 5 seconds, fully stop (reset joystick)
    coastingTimeout = setTimeout(() => {
        coastingTimeout = null;
    }, COAST_DURATION);
}

// ---------- Publish Twist messages at ~30Hz while active ----------
function startPublishing() {
    if (animationId) cancelAnimationFrame(animationId);
    function publishLoop() {
        if (active) {
            let linear = joyY * speedScale;
            let angular = joyX * speedScale * -Math.sign(linear);
            publishTwist(linear, angular);
            animationId = requestAnimationFrame(publishLoop);
        } else {
            animationId = null;
        }
    }
    publishLoop();
}

function publishTwist(linear, angular) {
    if (!ros.isConnected) return;
    const twist = new ROSLIB.Message({
        twist: {
            linear: { x: linear, y: 0, z: 0 },
            angular: { x: 0, y: 0, z: angular },
        }
    });
    cmdVelPub.publish(twist);

    // Update debug output
    document.getElementById('linearVal').textContent = linear.toFixed(3);
    document.getElementById('angularVal').textContent = angular.toFixed(3);
}

// ---------- Attach events (mouse + touch) ----------
canvas.addEventListener('mousedown', handleStart);
window.addEventListener('mousemove', handleMove);
window.addEventListener('mouseup', handleEnd);

canvas.addEventListener('touchstart', handleStart, { passive: false });
window.addEventListener('touchmove', handleMove, { passive: false });
window.addEventListener('touchend', handleEnd);

draw();