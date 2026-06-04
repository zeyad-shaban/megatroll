#include "megajaw_hardware_direct/GripperDriver.hpp"
#include <iostream>
#include <pigpiod_if2.h>

GripperDriver::GripperDriver(int servoPin)
    : pi(-1), _servoPin(servoPin)
{
    pi = pigpio_start(NULL, NULL);
    if (pi < 0) {
        std::cout << "Failed to connect to pigpiod" << std::endl;
    }

    set_mode(pi, _servoPin, PI_OUTPUT);

    _worker = std::thread(&GripperDriver::_workerLoop, this);
}

GripperDriver::~GripperDriver()
{
    {
        std::lock_guard<std::mutex> lock(_mutex);
        _stop = true;
        _cv.notify_all();
    }

    if (_worker.joinable()) {
        _worker.join();
    }

    std::cout << "Cleaning GPIO state..." << std::endl;
    pigpio_stop(pi);
}

void GripperDriver::setGripperPosition(double cmd)
{
    if (cmd > 0.5) {
        _openGripper();
    } else {
        _closeGripper();
    }
}

void GripperDriver::_openGripper()
{
    std::lock_guard<std::mutex> lock(_mutex);

    ++_generation; // invalidates older pending actions
    set_servo_pulsewidth(pi, _servoPin, 2000); // open

    _releasePending = true;
    _releaseAt = std::chrono::steady_clock::now() + std::chrono::milliseconds(500);

    _cv.notify_all();
}

void GripperDriver::_closeGripper()
{
    std::lock_guard<std::mutex> lock(_mutex);

    ++_generation;      // cancels any pending release
    _releasePending = false;

    set_servo_pulsewidth(pi, _servoPin, 1000); // close
    _cv.notify_all();
}

void GripperDriver::_workerLoop()
{
    std::unique_lock<std::mutex> lock(_mutex);

    while (!_stop) {
        if (!_releasePending) {
            _cv.wait(lock, [this] { return _stop || _releasePending; });
            continue;
        }

        auto targetTime = _releaseAt;
        int token = _generation;

        if (_cv.wait_until(lock, targetTime, [this, token] {
                return _stop || !_releasePending || token != _generation;
            })) {
            continue;
        }

        if (_stop) {
            break;
        }

        if (_releasePending && token == _generation && std::chrono::steady_clock::now() >= _releaseAt) {
            set_servo_pulsewidth(pi, _servoPin, 0); // release
            _releasePending = false;
        }
    }
}