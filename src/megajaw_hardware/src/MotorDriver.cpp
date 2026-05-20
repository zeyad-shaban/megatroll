#include "megajaw_hardware/MotorDriver.hpp"
#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <cerrno>

// Protocol constants matching STM32 firmware
constexpr uint8_t HEADER1 = 0xAA;
constexpr uint8_t HEADER2 = 0x55;
constexpr int16_t MAX_PWM = 100;

MotorDriver::MotorDriver(const std::string &serial_port, int baudrate)
    : serial_port_(serial_port), baudrate_(baudrate), serial_fd_(-1), left_speed_(0.0f), right_speed_(0.0f)
{

    if (!openSerialPort())
    {
        std::cerr << "Failed to open serial port: " << serial_port << std::endl;
        return;
    }

    std::cout << "MotorDriver initialized on " << serial_port << " at " << baudrate << " baud" << std::endl;
}

MotorDriver::~MotorDriver()
{
    std::cout << "Cleaning MotorDriver state..." << std::endl;
    stopMotors();
    closeSerialPort();
}

bool MotorDriver::openSerialPort()
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

    // Set timeout: 100ms inter-character timeout, 0.1s total read timeout
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

void MotorDriver::closeSerialPort()
{
    if (serial_fd_ != -1)
    {
        close(serial_fd_);
        serial_fd_ = -1;
    }
}

int16_t MotorDriver::scaleToPWM(float speedPerc)
{
    // Clamp input to [-1.0, 1.0]
    if (speedPerc < -1.0f)
    {
        speedPerc = -1.0f;
    }
    else if (speedPerc > 1.0f)
    {
        speedPerc = 1.0f;
    }

    // Scale to [-100, 100] range
    return static_cast<int16_t>(speedPerc * MAX_PWM);
}

void MotorDriver::sendToSTM32(int16_t left_pwm, int16_t right_pwm)
{
    if (serial_fd_ == -1)
    {
        return;
    }

    // Pack protocol: [0xAA][0x55][L_lo][L_hi][R_lo][R_hi]
    uint8_t message[6];
    message[0] = HEADER1;
    message[1] = HEADER2;
    message[2] = left_pwm & 0xFF;         // Low byte
    message[3] = (left_pwm >> 8) & 0xFF;  // High byte
    message[4] = right_pwm & 0xFF;        // Low byte
    message[5] = (right_pwm >> 8) & 0xFF; // High byte

    ssize_t bytes_written = write(serial_fd_, message, sizeof(message));
    if (bytes_written != sizeof(message))
    {
        std::cerr << "Error writing to serial port: " << strerror(errno) << std::endl;
    }
}

void MotorDriver::stopMotors()
{
    left_speed_ = 0.0f;
    right_speed_ = 0.0f;
    sendToSTM32(0, 0);
}

void MotorDriver::setLeftMotor(float speedPerc)
{
    left_speed_ = speedPerc;
    int16_t left_pwm = scaleToPWM(left_speed_);
    int16_t right_pwm = scaleToPWM(right_speed_);
    sendToSTM32(left_pwm, right_pwm);
}

void MotorDriver::setRightMotor(float speedPerc)
{
    right_speed_ = speedPerc;
    int16_t left_pwm = scaleToPWM(left_speed_);
    int16_t right_pwm = scaleToPWM(right_speed_);
    sendToSTM32(left_pwm, right_pwm);
}
