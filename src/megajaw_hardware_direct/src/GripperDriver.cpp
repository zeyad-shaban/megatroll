#include "megajaw_hardware_direct/GripperDriver.hpp"
#include <pigpiod_if2.h>
#include <iostream>

GripperDriver::GripperDriver(int servoPin)
    : _servoPin(servoPin)
{
    _pi = pigpio_start(NULL, NULL);
    if (_pi < 0) {
        std::cerr << "[Gripper] pigpiod connection failed" << std::endl;
        return;
    }
    set_mode(_pi, _servoPin, PI_OUTPUT);
    _worker = std::thread(&GripperDriver::_workerLoop, this);
}

GripperDriver::~GripperDriver()
{
    _stop = true;
    if (_worker.joinable())
        _worker.join();
    if (_pi >= 0) {
        set_servo_pulsewidth(_pi, _servoPin, 0);
        set_mode(_pi, _servoPin, PI_INPUT);
        pigpio_stop(_pi);
    }
}

void GripperDriver::setGripperPosition(double cmd)
{
    if (cmd > 0.5)
        _openGripper();
    else
        _closeGripper();
}

void GripperDriver::_openGripper()
{
    std::lock_guard<std::mutex> lock(_mutex);
    if (_pi < 0) return;

    // If the timer is already active, we are already open (or opening)
    if (_openTimerActive) {
        // Ignore repeated open commands – do not reset the timer
        return;
    }

    // Transition from closed to open
    set_servo_pulsewidth(_pi, _servoPin, 2000);   // open
    _openTimerActive = true;
    _releaseTime = std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
}

void GripperDriver::_closeGripper()
{
    std::lock_guard<std::mutex> lock(_mutex);
    if (_pi < 0) return;

    // Cancel any pending release
    if (_openTimerActive) {
        _openTimerActive = false;
    }
    set_servo_pulsewidth(_pi, _servoPin, 1000);   // close (holds)
}

void GripperDriver::_workerLoop()
{
    while (!_stop) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        if (!_openTimerActive)
            continue;

        auto now = std::chrono::steady_clock::now();
        if (now >= _releaseTime) {
            std::lock_guard<std::mutex> lock(_mutex);
            if (_openTimerActive) {  // double-check after lock
                // Stop PWM and tristate
                set_servo_pulsewidth(_pi, _servoPin, 0);
                set_mode(_pi, _servoPin, PI_INPUT);
                _openTimerActive = false;
            }
        }
    }
}