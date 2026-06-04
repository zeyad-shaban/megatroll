#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <thread>

class GripperDriver
{
public:
    GripperDriver(int servoPin);
    ~GripperDriver();

    void setGripperPosition(float pos);

private:
    int pi;
    int _servoPin;

    std::thread _worker;
    std::mutex _mutex;
    std::condition_variable _cv;
    bool _stop{false};

    bool _releasePending{false};
    std::chrono::steady_clock::time_point _releaseAt;
    int _generation{0};

    void _openGripper();
    void _closeGripper();
    void _workerLoop();
};