#include "megajaw_hardware_direct/GripperDriver.hpp"
#include <pigpiod_if2.h>
#include <iostream>
#include <iomanip>
#include <ctime>

static std::string now_str() {
    auto now = std::chrono::system_clock::now();
    auto now_c = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&now_c), "%H:%M:%S");
    return ss.str();
}

GripperDriver::GripperDriver(int servoPin)
    : _servoPin(servoPin)
{
    std::cout << "[" << now_str() << "] GripperDriver: constructor started, pin=" << servoPin << std::endl;

    _pi = pigpio_start(NULL, NULL);
    if (_pi < 0) {
        std::cerr << "[" << now_str() << "] GripperDriver: FAILED to connect to pigpiod (error code " << _pi << ")" << std::endl;
        return;
    }
    std::cout << "[" << now_str() << "] GripperDriver: pigpiod connected, pi=" << _pi << std::endl;

    if (set_mode(_pi, _servoPin, PI_OUTPUT) != 0) {
        std::cerr << "[" << now_str() << "] GripperDriver: failed to set pin mode to OUTPUT" << std::endl;
    } else {
        std::cout << "[" << now_str() << "] GripperDriver: pin " << _servoPin << " set as output" << std::endl;
    }

    _worker = std::thread(&GripperDriver::_workerLoop, this);
    std::cout << "[" << now_str() << "] GripperDriver: worker thread started" << std::endl;
}

GripperDriver::~GripperDriver()
{
    std::cout << "[" << now_str() << "] GripperDriver: destructor called" << std::endl;
    _stop = true;

    if (_worker.joinable()) {
        std::cout << "[" << now_str() << "] GripperDriver: waiting for worker thread to join..." << std::endl;
        _worker.join();
        std::cout << "[" << now_str() << "] GripperDriver: worker thread joined" << std::endl;
    }

    if (_pi >= 0) {
        std::cout << "[" << now_str() << "] GripperDriver: stopping PWM and tristating pin " << _servoPin << std::endl;
        set_servo_pulsewidth(_pi, _servoPin, 0);
        set_mode(_pi, _servoPin, PI_INPUT);
        pigpio_stop(_pi);
        std::cout << "[" << now_str() << "] GripperDriver: pigpio stopped" << std::endl;
    }
}

void GripperDriver::setGripperPosition(double cmd)
{
    std::cout << "[" << now_str() << "] setGripperPosition(" << cmd << ")" << std::endl;
    if (cmd > 0.5)
        _openGripper();
    else
        _closeGripper();
}

void GripperDriver::_openGripper()
{
    std::lock_guard<std::mutex> lock(_mutex);
    std::cout << "[" << now_str() << "] _openGripper: called" << std::endl;

    if (_pi < 0) {
        std::cerr << "[" << now_str() << "] _openGripper: pigpio not available, aborting" << std::endl;
        return;
    }

    std::cout << "[" << now_str() << "] _openGripper: setting servo pulse to 2000 (open) on pin " << _servoPin << std::endl;
    set_servo_pulsewidth(_pi, _servoPin, 2000);

    _releasePending = true;
    _releaseTime = std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
    std::cout << "[" << now_str() << "] _openGripper: release scheduled in 500 ms (releasePending=true)" << std::endl;
}

void GripperDriver::_closeGripper()
{
    std::lock_guard<std::mutex> lock(_mutex);
    std::cout << "[" << now_str() << "] _closeGripper: called" << std::endl;

    if (_pi < 0) {
        std::cerr << "[" << now_str() << "] _closeGripper: pigpio not available, aborting" << std::endl;
        return;
    }

    if (_releasePending) {
        std::cout << "[" << now_str() << "] _closeGripper: cancelling pending auto-release (was active)" << std::endl;
    }
    _releasePending = false;
    std::cout << "[" << now_str() << "] _closeGripper: setting servo pulse to 1000 (close/hold) on pin " << _servoPin << std::endl;
    set_servo_pulsewidth(_pi, _servoPin, 1000);
}

void GripperDriver::_workerLoop()
{
    std::cout << "[" << now_str() << "] WorkerLoop: thread started" << std::endl;

    while (!_stop) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        if (!_releasePending)
            continue;

        // we have a pending release – check if it's time
        {
            std::lock_guard<std::mutex> lock(_mutex);
            auto now = std::chrono::steady_clock::now();
            if (_releasePending && now >= _releaseTime) {
                std::cout << "[" << now_str() << "] WorkerLoop: release time reached, stopping servo on pin " << _servoPin << std::endl;
                if (_pi >= 0) {
                    // 1) Stop PWM
                    set_servo_pulsewidth(_pi, _servoPin, 0);
                    std::cout << "[" << now_str() << "] WorkerLoop: PWM set to 0" << std::endl;
                    // 2) Tristate the pin to physically release the servo
                    set_mode(_pi, _servoPin, PI_INPUT);
                    std::cout << "[" << now_str() << "] WorkerLoop: pin set to INPUT (tristate)" << std::endl;
                } else {
                    std::cerr << "[" << now_str() << "] WorkerLoop: pi invalid, cannot release" << std::endl;
                }
                _releasePending = false;
                std::cout << "[" << now_str() << "] WorkerLoop: release completed, releasePending=false" << std::endl;
            }
        }
    }

    std::cout << "[" << now_str() << "] WorkerLoop: thread exiting (_stop=true)" << std::endl;
}