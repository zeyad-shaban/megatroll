// include/megajaw_hardware_direct/MotorDriver.hpp
#pragma once

#include <string>

class MotorDriver
{
public:
    MotorDriver(int rpA, int rpB, int lpA, int lpB);
    ~MotorDriver();

    void stopMotors();
    void setLeftMotor(float speedPerc);
    void setRightMotor(float speedPerc);

private:
    void setMotors(float speedPerc, int pinA, int pinB);

    int pi_;
    int _rpA, _rpB, _lpA, _lpB;
};