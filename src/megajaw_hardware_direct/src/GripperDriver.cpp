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

    if (_openTimerActive) {
        // Already open (timer running) – ignore repeated open
        std::cout << "[Gripper] open ignored (already opening)" << std::endl;
        return;
    }

    // Transition from closed (or released) to open-powered
    std::cout << "[Gripper] open -> sending 2000 us, timer started (500ms)" << std::endl;
    set_servo_pulsewidth(_pi, _servoPin, 2000);
    _openTimerActive = true;
    _releaseTime = std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
}

void GripperDriver::_closeGripper()
{
    std::lock_guard<std::mutex> lock(_mutex);
    if (_pi < 0) return;

    if (_openTimerActive) {
        std::cout << "[Gripper] close -> cancelling timer, sending 1000 us (hold)" << std::endl;
        _openTimerActive = false;
    } else {
        std::cout << "[Gripper] close -> sending 1000 us (hold)" << std::endl;
    }
    set_servo_pulsewidth(_pi, _servoPin, 1000);
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
            if (_openTimerActive) {
                std::cout << "[Gripper] timer expired -> releasing (PWM=0, tristate)" << std::endl;
                set_servo_pulsewidth(_pi, _servoPin, 0);
                set_mode(_pi, _servoPin, PI_INPUT);
                _openTimerActive = false;
            }
        }
    }
}