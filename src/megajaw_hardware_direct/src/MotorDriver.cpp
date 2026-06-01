#include "megajaw_hardware_direct/MotorDriver.hpp"
#include <iostream>
#include <pigpiod_if2.h>
#include <unistd.h>

MotorDriver::MotorDriver(int rpA, int rpB, int lpA, int lpB) {
    pi = pigpio_start(NULL, NULL);   // connect to pigpiod
    if (pi < 0) {
        std::cout << "Failed to connect to pigpiod" << std::endl;
    } else {
        set_mode(pi, lpA, PI_OUTPUT);
        set_mode(pi, lpB, PI_OUTPUT);
        set_mode(pi, rpA, PI_OUTPUT);
        set_mode(pi, rpB, PI_OUTPUT);
    }
    _rpA = rpA; _rpB = rpB; _lpA = lpA; _lpB = lpB;
}

MotorDriver::~MotorDriver() {
    std::cout << "Cleaning GPIO state..." << std::endl;
    stopMotors();
    pigpio_stop(pi);   // disconnect, daemon keeps running
}

void MotorDriver::stopMotors() {
    setMotors(0, _lpA, _lpB);
    setMotors(0, _rpA, _rpB);
}

void MotorDriver::setLeftMotor(float speedPerc) {
    setMotors(speedPerc, _lpA, _lpB);
}

void MotorDriver::setRightMotor(float speedPerc) {
    setMotors(speedPerc, _rpA, _rpB);
}

void MotorDriver::setMotors(float speedPerc, int pinA, int pinB) {
    if (speedPerc < -1) {
        // std::cout << "Warning: speedPerc can't be below -1, got " << speedPerc << std::endl;
        speedPerc = -1;
    } else if(speedPerc > 1) {
        // std::cout << "Warning: speedPerc can't be above 1, got " << speedPerc << std::endl;
        speedPerc = 1;
    }

    if (speedPerc >= 0) {
        set_PWM_dutycycle(pi, pinA, 0);
        set_PWM_dutycycle(pi, pinB, static_cast<int>(speedPerc * 255));
    } else {
        set_PWM_dutycycle(pi, pinB, 0);
        set_PWM_dutycycle(pi, pinA, static_cast<int>(-speedPerc * 255));
    }
}