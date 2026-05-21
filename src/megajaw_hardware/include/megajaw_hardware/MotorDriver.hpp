#ifndef MOTOR_DRIVER_HPP  
#define MOTOR_DRIVER_HPP  
  
#include <string>  
#include <cstdint>  
  
class MotorDriver {  
public:  
    MotorDriver(const std::string& serial_port, int baudrate);  
    ~MotorDriver();  
  
    void stopMotors();  
    void setLeftMotor(float speedPerc);  
    void setRightMotor(float speedPerc);  
  
private:  
    int serial_fd_;  
    std::string serial_port_;  
    int baudrate_;  
    float left_speed_;  
    float right_speed_;  
      
    bool openSerialPort();  
    void closeSerialPort();  
    void sendToSTM32(int16_t left_pwm, int16_t right_pwm);  
    int16_t scaleToPWM(float speedPerc);  
};  
  
#endif
