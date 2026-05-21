#include "megajaw_hardware/GripperDriver.hpp"
#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <cerrno>

// Protocol constants matching STM32 firmware
constexpr uint8_t GRIPPER_HEADER1 = 0xAB;
constexpr uint8_t GRIPPER_HEADER2 = 0xCD;
constexpr uint8_t GRIPPER_OPEN = 0x00;
constexpr uint8_t GRIPPER_CLOSE = 0x01;

GripperDriver::GripperDriver(const std::string &serial_port, int baudrate)
    : serial_port_(serial_port), baudrate_(baudrate), serial_fd_(-1)
{
    if (!openSerialPort())
    {
        std::cerr << "Failed to open serial port: " << serial_port << std::endl;
        return;
    }

    std::cout << "GripperDriver initialized on " << serial_port << " at " << baudrate << " baud" << std::endl;
}

GripperDriver::~GripperDriver()
{
    std::cout << "Cleaning GripperDriver state..." << std::endl;
    closeSerialPort();
}

bool GripperDriver::openSerialPort()
{
    serial_fd_ = open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (serial_fd_ == -1)
    {
        std::cerr << "Error opening serial port " << serial_port_ << ": " << strerror(errno) << std::endl;
        return false;
    }

    struct termios options;
    tcgetattr(serial_fd_, &options);

    // Set baud rate
    speed_t baud;
    switch (baudrate_)
    {
    case 9600:
        baud = B9600;
        break;
    case 19200:
        baud = B19200;
        break;
    case 38400:
        baud = B38400;
        break;
    case 57600:
        baud = B57600;
        break;
    case 115200:
        baud = B115200;
        break;
    default:
        std::cerr << "Unsupported baud rate: " << baudrate_ << std::endl;
        close(serial_fd_);
        serial_fd_ = -1;
        return false;
    }

    cfsetispeed(&options, baud);
    cfsetospeed(&options, baud);

    // 8N1 configuration
    options.c_cflag &= ~PARENB;        // No parity
    options.c_cflag &= ~CSTOPB;        // 1 stop bit
    options.c_cflag &= ~CSIZE;         // Clear data size bits
    options.c_cflag |= CS8;            // 8 data bits
    options.c_cflag &= ~CRTSCTS;       // No hardware flow control
    options.c_cflag |= CREAD | CLOCAL; // Enable receiver, ignore modem control lines

    // Raw input mode
    options.c_iflag &= ~(IXON | IXOFF | IXANY | IGNBRK | INLCR | ICRNL);
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_oflag &= ~OPOST;

    // Set timeout
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 1;

    if (tcsetattr(serial_fd_, TCSANOW, &options) != 0)
    {
        std::cerr << "Error setting serial port attributes: " << strerror(errno) << std::endl;
        close(serial_fd_);
        serial_fd_ = -1;
        return false;
    }

    // Flush buffers
    tcflush(serial_fd_, TCIOFLUSH);

    return true;
}

void GripperDriver::closeSerialPort()
{
    if (serial_fd_ != -1)
    {
        close(serial_fd_);
        serial_fd_ = -1;
    }
}

void GripperDriver::sendGripperCommand(uint8_t cmd)
{
    if (serial_fd_ == -1)
    {
        std::cerr << "Serial port not open, cannot send gripper command" << std::endl;
        return;
    }

    // Pack protocol: [0xAB][0xCD][cmd]
    uint8_t message[3];
    message[0] = GRIPPER_HEADER1;
    message[1] = GRIPPER_HEADER2;
    message[2] = cmd;

    ssize_t bytes_written = write(serial_fd_, message, sizeof(message));
    if (bytes_written != sizeof(message))
    {
        std::cerr << "Error writing to serial port: " << strerror(errno) << std::endl;
    }
}

void GripperDriver::openGripper()
{
    std::cout << "Opening gripper..." << std::endl;
    sendGripperCommand(GRIPPER_OPEN);
}

void GripperDriver::closeGripper()
{
    std::cout << "Closing gripper..." << std::endl;
    sendGripperCommand(GRIPPER_CLOSE);
}
