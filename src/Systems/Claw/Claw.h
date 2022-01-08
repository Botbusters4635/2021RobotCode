
//
// Created by hiram on 9/14/19.
//

#ifndef BOTBUSTERS_REBIRTH_CLAW_H
#define BOTBUSTERS_REBIRTH_CLAW_H

#include <Core/EctoModule/System.h>
#include <Core/PCM/PCMManager.h>
#include <Core/EctoInput/InputManager.h>
#include <Core/MotorHandler/EctoMotor/EctoMotor.h>
#include <Core/MotorHandler/MotorManager.h>
#include <Core/MotorHandler/EctoMotor/EctoTalon.h>
#include <Core/EctoInput/Axis/JoystickAxisExpo.h>
#include <Core/EctoInput/Buttons/EctoButton.h>
#include <Sensors/EctoPotentiometer.h>
#include <Control/EctoMagic/EctoMagic.h>
#include <mutex>
#include <thread>

enum class AutomaticIntakeStatus {
	Timeout,
	Finished,
	Running,
	Stopped
};

class Claw : public System {
public:
	Claw(const std::shared_ptr<EctoMotor> &leftClawMotor, const std::shared_ptr<EctoMotor> &rightClawMotor,
	     const std::shared_ptr<EctoMotor> &angleMotor);
	
	void robotInit() override;
	
	void robotUpdate() override;
	
	void setClawMotors(double leftValue, double rightValue);
	
	void setClawMotors(double value);
	
	void setClawAngle(double angle);
	
	double getCurrentAngle() const;
	
	void setPistonState(bool state);
	
	bool getCurrentPistonState() const;
	
	void disableClaw(bool disabled);

private:
	PCMManager &pcm = PCMManager::getInstance();
	
	MotionProfileConfig motionProfileConfig;
	
	bool currentPistonState = false;
	double currentLeftValue, currentRightValue;
	
	const std::string clawPistonName = "clawPiston";
	std::shared_ptr<frc::DoubleSolenoid> clawPiston;
	
	std::shared_ptr<EctoMotor> rightClawMotor, leftClawMotor, angleMotor;
	
	std::unique_ptr<EctoMagic> magicAngleController;
	std::unique_ptr<EctoPID> angleController;
	
	EctoPotentiometer angleSource = EctoPotentiometer{0, -M_PI, 0.96, 4.15, -4.15};
	const double feedForwardClaw = 0.4;
	
	PIDConfig anglePIDConfig;
	
	double angleTarget = 0.1;
	std::mutex angleTargetLock;
	std::unique_ptr<std::thread> anglePIDThread;
	
	void updatePID();
	
	std::mutex disabledLock;
	bool isDisabled = false;
};


#endif //BOTBUSTERS_REBIRTH_CLAW_H
