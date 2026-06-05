#pragma once

#include <atomic>
#include <chrono>
#include <thread>
#include <mutex>

class GripperDriver
{
public:
    GripperDriver(int servoPin);
    ~GripperDriver();

    void setGripperPosition(double cmd);

private:
    int _servoPin;
    int _pi;

    std::atomic<bool> _stop{false};
    std::atomic<bool> _openTimerActive{false};
    std::atomic<bool> _released{false};
    std::chrono::steady_clock::time_point _releaseTime;
    std::thread _worker;
    std::mutex _mutex;

    void _openGripper();
    void _closeGripper();
    void _workerLoop();
};