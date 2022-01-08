//
// Created by alberto on 16/09/19.
//

#ifndef BOTBUSTERS_REBIRTH_BALLINTAKE_H
#define BOTBUSTERS_REBIRTH_BALLINTAKE_H

#include <Core/EctoModule/System.h>
#include <Core/MotorHandler/EctoMotor/EctoTalon.h>
#include <Core/PCM/PCMManager.h>
#include <Core/EctoInput/InputManager.h>
#include <Core/EctoInput/Axis/JoystickAxisExpo.h>

class BallIntake : public System {
public:
	BallIntake(const std::shared_ptr<EctoMotor> &leftMotor, const std::shared_ptr<EctoMotor> &rightMotor);
	
	void robotInit() override;
	
	void robotUpdate() override;
	
	void setLeftIntakeState(bool state, double setpoint);
	
	void setRightIntakeState(bool state, double setpoint);

private:
	std::shared_ptr<frc::DoubleSolenoid> leftPiston, rightPiston;
	
	PCMManager &pcm = PCMManager::getInstance();
	
	std::shared_ptr<EctoMotor> leftIntakeMotor, rightIntakeMotor;
	
	bool isIntakeLifted = true;
	
	double lastSetpoint = 0;
	bool lastPistonState;
};


#endif //BOTBUSTERS_REBIRTH_BALLINTAKE_H
