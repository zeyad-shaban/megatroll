#ifndef GRIPPER_DRIVER_HPP
#define GRIPPER_DRIVER_HPP

#include <string>
#include <cstdint>

class GripperDriver
{
public:
    GripperDriver(const std::string &serial_port, int baudrate);
    ~GripperDriver();

    void openGripper();
    void closeGripper();

private:
    int serial_fd_;
    std::string serial_port_;
    int baudrate_;

    bool openSerialPort();
    void closeSerialPort();
    void sendGripperCommand(uint8_t cmd);
};

#endif
