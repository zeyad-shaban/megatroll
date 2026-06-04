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

    if (_released || _openTimerActive)
        return;   // already open (with or without power)

    set_servo_pulsewidth(_pi, _servoPin, 2000);
    _openTimerActive = true;
    _released = false;
    _releaseTime = std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
}

void GripperDriver::_closeGripper()
{
    std::lock_guard<std::mutex> lock(_mutex);
    if (_pi < 0) return;

    _openTimerActive = false;
    _released = false;
    set_servo_pulsewidth(_pi, _servoPin, 1000);
}

void GripperDriver::_workerLoop()
{
    while (!_stop) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        if (!_openTimerActive)
            continue;

        if (std::chrono::steady_clock::now() >= _releaseTime) {
            std::lock_guard<std::mutex> lock(_mutex);
            if (_openTimerActive) {
                set_servo_pulsewidth(_pi, _servoPin, 0);
                set_mode(_pi, _servoPin, PI_INPUT);
                _openTimerActive = false;
                _released = true;
            }
        }
    }
}