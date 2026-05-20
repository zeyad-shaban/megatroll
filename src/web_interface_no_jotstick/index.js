// ROS 2 browser controls for a two-wheel differential-drive car.

const ROS_URL = 'ws://zeyadcodepi.local:9090';
const PUBLISH_HZ = 20;
const RECONNECT_MS = 2000;
const ZERO_BURST_COUNT = 4;

const statusDiv = document.getElementById('statusMsg');
const commandPad = document.getElementById('commandPad');
const powerSlider = document.getElementById('powerScale');
const powerValue = document.getElementById('powerValue');
const leftWheelSlider = document.getElementById('leftWheel');
const rightWheelSlider = document.getElementById('rightWheel');
const leftSliderValue = document.getElementById('leftSliderValue');
const rightSliderValue = document.getElementById('rightSliderValue');
const stopButton = document.getElementById('stopButton');
const linearOut = document.getElementById('linearVal');
const angularOut = document.getElementById('angularVal');
const leftOut = document.getElementById('leftVal');
const rightOut = document.getElementById('rightVal');

let reconnectTimer = null;
let publishTimer = null;
let activeCommand = null;
let activePointerId = null;
let zeroBurstRemaining = ZERO_BURST_COUNT;
let manualLeft = 0;
let manualRight = 0;

const ros = new ROSLIB.Ros({ url: ROS_URL });
const cmdVelTopic = new ROSLIB.Topic({
    ros,
    name: '/cmd_vel',
    messageType: 'geometry_msgs/TwistStamped',
});

const keyCommands = new Map([
    ['ArrowUp', 'forward'],
    ['w', 'forward'],
    ['W', 'forward'],
    ['ArrowDown', 'backward'],
    ['s', 'backward'],
    ['S', 'backward'],
    ['ArrowLeft', 'left'],
    ['a', 'left'],
    ['A', 'left'],
    ['ArrowRight', 'right'],
    ['d', 'right'],
    ['D', 'right'],
]);

function clamp(value, min, max) {
    return Math.max(min, Math.min(max, value));
}

function setStatus(kind, text) {
    statusDiv.className = `status ${kind}`;
    statusDiv.textContent = text;
}

function setControlsDisabled(disabled) {
    commandPad.classList.toggle('is-disabled', disabled);
    powerSlider.disabled = disabled;
    leftWheelSlider.disabled = disabled;
    rightWheelSlider.disabled = disabled;
    stopButton.disabled = disabled;
    document.querySelectorAll('.drive-button[data-command]').forEach((button) => {
        button.disabled = disabled;
    });
}

function setConnected(ok) {
    setControlsDisabled(!ok);

    if (ok) {
        if (reconnectTimer) {
            clearInterval(reconnectTimer);
            reconnectTimer = null;
        }
        setStatus('connected', 'Connected');
        startPublishing();
        return;
    }

    activeCommand = null;
    activePointerId = null;
    resetManualSliders();
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

function commandToWheels(command) {
    const power = Number(powerSlider.value);

    switch (command) {
        case 'forward':
            return { left: power, right: power };
        case 'backward':
            return { left: -power, right: -power };
        case 'left':
            return { left: power, right: -power };
        case 'right':
            return { left: -power, right: power };
        default:
            return { left: 0, right: 0 };
    }
}

function wheelsToTwist(leftPercent, rightPercent) {
    const left = clamp(leftPercent, -100, 100) / 100;
    const right = clamp(rightPercent, -100, 100) / 100;

    return {
        linear: clamp((left + right) / 2, -1, 1),
        angular: clamp((right - left) / 2, -1, 1),
        leftPercent: Math.round(left * 100),
        rightPercent: Math.round(right * 100),
    };
}

function currentWheels() {
    if (activeCommand) return commandToWheels(activeCommand);
    return { left: manualLeft, right: manualRight };
}

function updateReadout(linear, angular, leftPercent, rightPercent) {
    linearOut.textContent = linear.toFixed(2);
    angularOut.textContent = angular.toFixed(2);
    leftOut.textContent = `${leftPercent}%`;
    rightOut.textContent = `${rightPercent}%`;
}

function publishCurrentCommand() {
    const { left, right } = currentWheels();
    const { linear, angular, leftPercent, rightPercent } = wheelsToTwist(left, right);
    const shouldSend = activeCommand || left !== 0 || right !== 0 || zeroBurstRemaining > 0;

    if (!shouldSend) {
        updateReadout(0, 0, 0, 0);
        return;
    }

    publishCmd(linear, angular);
    updateReadout(linear, angular, leftPercent, rightPercent);

    if (!activeCommand && left === 0 && right === 0) zeroBurstRemaining -= 1;
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
    updateReadout(0, 0, 0, 0);
}

function setActiveButton(command) {
    document.querySelectorAll('.drive-button[data-command]').forEach((button) => {
        button.classList.toggle('is-active', button.dataset.command === command);
    });
}

function beginCommand(command, pointerId = null) {
    if (!ros.isConnected) return;
    resetManualSliders();
    activeCommand = command;
    activePointerId = pointerId;
    zeroBurstRemaining = ZERO_BURST_COUNT;
    setActiveButton(command);
    publishCurrentCommand();
}

function endCommand() {
    if (!activeCommand) return;
    activeCommand = null;
    activePointerId = null;
    zeroBurstRemaining = ZERO_BURST_COUNT;
    setActiveButton(null);
    publishCmd(0, 0);
    updateReadout(0, 0, 0, 0);
}

function resetManualSliders() {
    manualLeft = 0;
    manualRight = 0;
    leftWheelSlider.value = '0';
    rightWheelSlider.value = '0';
    leftSliderValue.textContent = '0%';
    rightSliderValue.textContent = '0%';
}

function stopAll() {
    activeCommand = null;
    activePointerId = null;
    zeroBurstRemaining = ZERO_BURST_COUNT;
    setActiveButton(null);
    resetManualSliders();
    publishCmd(0, 0);
    updateReadout(0, 0, 0, 0);
}

document.querySelectorAll('.drive-button[data-command]').forEach((button) => {
    button.addEventListener('pointerdown', (event) => {
        if (button.disabled) return;
        event.preventDefault();
        button.setPointerCapture(event.pointerId);
        beginCommand(button.dataset.command, event.pointerId);
    });

    button.addEventListener('pointerup', (event) => {
        if (event.pointerId !== activePointerId) return;
        event.preventDefault();
        endCommand();
    });

    button.addEventListener('pointercancel', (event) => {
        if (event.pointerId === activePointerId) endCommand();
    });

    button.addEventListener('lostpointercapture', () => {
        if (button.dataset.command === activeCommand) endCommand();
    });
});

powerSlider.addEventListener('input', () => {
    powerValue.textContent = `${powerSlider.value}%`;
    if (activeCommand) publishCurrentCommand();
});

leftWheelSlider.addEventListener('input', () => {
    activeCommand = null;
    setActiveButton(null);
    manualLeft = Number(leftWheelSlider.value);
    leftSliderValue.textContent = `${manualLeft}%`;
    zeroBurstRemaining = ZERO_BURST_COUNT;
    publishCurrentCommand();
});

rightWheelSlider.addEventListener('input', () => {
    activeCommand = null;
    setActiveButton(null);
    manualRight = Number(rightWheelSlider.value);
    rightSliderValue.textContent = `${manualRight}%`;
    zeroBurstRemaining = ZERO_BURST_COUNT;
    publishCurrentCommand();
});

stopButton.addEventListener('click', stopAll);

window.addEventListener('keydown', (event) => {
    if (event.repeat || !keyCommands.has(event.key)) return;
    event.preventDefault();
    beginCommand(keyCommands.get(event.key));
});

window.addEventListener('keyup', (event) => {
    if (!keyCommands.has(event.key)) return;
    event.preventDefault();
    endCommand();
});

window.addEventListener('blur', stopAll);

setControlsDisabled(true);
setConnected(false);
powerValue.textContent = `${powerSlider.value}%`;
updateReadout(0, 0, 0, 0);
