#pragma once

#include <string>

class GripperDriver
{
public:
    GripperDriver(int servoPin);
    ~GripperDriver();

    void setGripperPosition(float pos);

private:
    int pi;
    int _servoPin;

    void _openGripper();
    void _closeGripper();
};