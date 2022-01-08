#include "Claw.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include "Core/MotorHandler/EctoMotor/DataTypes/EctoMotorMode.h"

//
// Created by hiram on 9/14/19.
//

Claw::Claw(const std::shared_ptr<EctoMotor> &leftClawMotor, const std::shared_ptr<EctoMotor> &rightClawMotor,
           const std::shared_ptr<EctoMotor> &angleMotor) : System("Claw") {
	this->rightClawMotor = rightClawMotor;
	this->leftClawMotor = leftClawMotor;
	this->angleMotor = angleMotor;
	
	/**
	 * Motor params
	 */
	leftClawMotor->invertMotor(true);
	
	leftClawMotor->enableBrakingOnIdle(true);
	rightClawMotor->enableBrakingOnIdle(true);
	
	leftClawMotor->enableCurrentLimit(true);
	leftClawMotor->setMotorCurrentLimit(40);
	rightClawMotor->enableCurrentLimit(true);
	rightClawMotor->setMotorCurrentLimit(40);
	
	rightClawMotor->setOpenLoopRampRate(0.1);
	leftClawMotor->setOpenLoopRampRate(0.1);
	
	angleMotor->invertMotor(true);
	angleMotor->setOpenLoopRampRate(0.05);
	
	/**
	 * Angle PID Config
	 */
	anglePIDConfig = PIDConfig();
	anglePIDConfig.p = 1.434;
	anglePIDConfig.d = 0.0002;
	angleController = std::make_unique<EctoPID>(angleSource, *angleMotor, anglePIDConfig);
	
	/**
	 * Motion profile config
	 */
	motionProfileConfig.maximumVelocity = 9.566;
	motionProfileConfig.maximumAcceleration = 4.2006;
	magicAngleController = std::make_unique<EctoMagic>(*angleController, motionProfileConfig);
}

void Claw::robotInit() {
	anglePIDThread = std::make_unique<std::thread>(std::bind(&Claw::updatePID, this));
	clawPiston = pcm.getPiston(clawPistonName);
	
	/**
	 * NT Init
	 */
	frc::SmartDashboard::PutNumber("Claw/PID/P", anglePIDConfig.p);
	frc::SmartDashboard::PutNumber("Claw/PID/I", anglePIDConfig.i);
	frc::SmartDashboard::PutNumber("Claw/PID/D", anglePIDConfig.d);
	
	frc::SmartDashboard::PutNumber("Claw/MotionProfile/MaximumVelocity", motionProfileConfig.maximumVelocity);
	frc::SmartDashboard::PutNumber("Claw/MotionProfile/MaximumAcceleration", motionProfileConfig.maximumAcceleration);
	
}

void Claw::updatePID() {
	for (;;) {
		angleTargetLock.lock();
		magicAngleController->setSetpoint(angleTarget);
		
		angleTargetLock.unlock();
		
		bool disable = false;
		
		disabledLock.lock();
		disable = isDisabled;
		disabledLock.unlock();
		
		if (disable) {
			angleMotor->set(0, MotorControlMode::Percent);
		} else {
			angleMotor->setControlMode(MotorControlMode::Position);
			double currentFeedforward = std::sin(-angleSource.getPosition() + 1.57) * feedForwardClaw;
			anglePIDConfig.f = currentFeedforward + 0.2;
			angleController->setConfig(anglePIDConfig);
			
			magicAngleController->update();
		}
		
		std::this_thread::sleep_for(std::chrono::milliseconds(20));
	}
}

void Claw::robotUpdate() {
	frc::SmartDashboard::PutNumber("Claw/ClawAngle", getCurrentAngle());
	
	frc::SmartDashboard::PutNumber("Claw/LeftMotorCurrent", leftClawMotor->getCurrent());
	frc::SmartDashboard::PutNumber("Claw/RightMotorCurrent", leftClawMotor->getCurrent());
	
	frc::SmartDashboard::PutNumber("Claw/LeftMotorSetPoint", currentLeftValue);
	frc::SmartDashboard::PutNumber("Claw/RightMotorSetPoint", currentRightValue);
	
	frc::SmartDashboard::PutBoolean("Claw/CurrentPistonState", getCurrentPistonState());
	
	/**
	 * Update configs
	 */
	PIDConfig newPIDClaw = anglePIDConfig;
	newPIDClaw.p = frc::SmartDashboard::GetNumber("Claw/PID/P", anglePIDConfig.p);
	newPIDClaw.i = frc::SmartDashboard::GetNumber("Claw/PID/I", anglePIDConfig.i);
	newPIDClaw.d = frc::SmartDashboard::GetNumber("Claw/PID/D", anglePIDConfig.d);
	
	if (newPIDClaw != anglePIDConfig) {
		angleController->setConfig(newPIDClaw);
		anglePIDConfig = newPIDClaw;
	}
	
	MotionProfileConfig newMotionProfileConfig;
	newMotionProfileConfig.maximumVelocity = frc::SmartDashboard::GetNumber("Claw/MotionProfile/MaximumVelocity",
	                                                                        motionProfileConfig.maximumVelocity);
	newMotionProfileConfig.maximumAcceleration = frc::SmartDashboard::GetNumber(
			"Claw/MotionProfile/MaximumAcceleration", motionProfileConfig.maximumAcceleration);
	
	if (newMotionProfileConfig != motionProfileConfig) {
		magicAngleController->setMotionProfileConfig(newMotionProfileConfig);
		motionProfileConfig = newMotionProfileConfig;
	}
}

void Claw::setClawMotors(double leftValue, double rightValue) {
	if (leftValue != currentLeftValue) {
		leftClawMotor->set(leftValue, MotorControlMode::Percent);
		currentLeftValue = leftValue;
	}
	
	if (rightValue != currentRightValue) {
		rightClawMotor->set(rightValue, MotorControlMode::Percent);
		currentRightValue = rightValue;
	}
}

void Claw::setClawMotors(double value) {
	setClawMotors(value, value);
}

void Claw::setClawAngle(double angle) {
	angleTargetLock.lock();
	angleTarget = angle;
	angleTargetLock.unlock();
}

double Claw::getCurrentAngle() const {
	return angleSource.getPosition();
}

void Claw::setPistonState(bool state) {
	if (state == currentPistonState) {
		return;
	}
	
	clawPiston->Set(state ? frc::DoubleSolenoid::Value::kForward : frc::DoubleSolenoid::Value::kReverse);
	currentPistonState = state;
}

bool Claw::getCurrentPistonState() const {
	return currentPistonState;
}

void Claw::disableClaw(bool disabled) {
	disabledLock.lock();
	isDisabled = disabled;
	disabledLock.unlock();
}