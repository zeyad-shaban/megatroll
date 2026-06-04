#include "megajaw_hardware_direct/GripperDriver.hpp"
#include <iostream>
#include <pigpiod_if2.h>
#include <unistd.h>

GripperDriver::GripperDriver(int servoPin) {
    pi = pigpio_start(NULL, NULL);
    if (pi < 0) {
        std::cout << "Failed to connect to pigpiod" << std::endl;
    }
    
    set_mode(pi, servoPin, PI_OUTPUT);
    _servoPin = servoPin;
}

Gripper::setGripperPosition(float pos) {
    std::cout << "Setting position to " << pos << std::endl;
}
GripperDriver::_openGripper() {
    set_servo_pulsewidth(pi, _servoPin, 2000); // open position
    std::this_thread::sleep_for(std::chrono::seconds(1));
    set_servo_pulsewidth(pi, _servoPin, 0); // release
}

GripperDriver::_closeGripper() {
    set_servo_pulsewidth(pi, _servoPin, 1000); // close position
}

GripperDriver::~GripperDriver() {
    std::cout << "Cleaning GPIO state..." << std::endl;
    pigpio_stop(pi);
}