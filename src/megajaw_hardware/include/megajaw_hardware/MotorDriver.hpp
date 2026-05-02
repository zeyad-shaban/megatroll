#ifndef MOTOR_DRIVER_HPP
#define MOTOR_DRIVER_HPP


class MotorDriver {
public:
	MotorDriver(int rpA, int rpB, int lpA, int lpB);		
	~MotorDriver();

	void stopMotors();
	void setLeftMotor(float speedPerc);
	void setRightMotor(float speedPerc);
private:
	int pi;
	int _rpA, _rpB, _lpA, _lpB;

	void setMotors(float speedPerc, int pina, int pinb);
};

#endif
