//
// Created by abiel on 9/20/19.
//

#include "Control/EctoMagic/EctoMagic.h"

EctoMagic::EctoMagic(EctoController &controller, const MotionProfileConfig &motionProfileConfig) : baseController(
		controller), motionProfile(motionProfileConfig) {
	this->motionProfileConfig = motionProfileConfig;
	
	this->motionProfileConfig.initialPosition = baseController.getControllerSource().getPosition();
	this->motionProfileConfig.initialVelocity = baseController.getControllerSource().getVelocity();
	
	this->motionProfileConfig.finalPosition = 0;
	
	startTime = std::chrono::high_resolution_clock::now();
	
	auto timePoint = std::chrono::high_resolution_clock::now();
	this->motionProfileConfig.startTime = std::chrono::duration<double>(timePoint - startTime).count();
}

void EctoMagic::setSetpoint(double setpoint) {
	if (setpoint != motionProfileConfig.finalPosition) {
		motionProfileConfig.initialPosition = baseController.getControllerSource().getPosition();
		motionProfileConfig.initialVelocity = baseController.getControllerSource().getVelocity();
		
		motionProfileConfig.finalPosition = setpoint;
		
		const auto timePoint = std::chrono::high_resolution_clock::now();
		motionProfileConfig.startTime = std::chrono::duration<double>(timePoint - startTime).count();
		
		motionProfile = TrapezoidalMotionProfile(motionProfileConfig);
	}
}

double EctoMagic::getSetpoint() const {
	return motionProfileConfig.finalPosition;
}

double EctoMagic::getError() const {
	return motionProfileConfig.finalPosition - baseController.getControllerSource().getPosition();
}

void EctoMagic::update() {
	auto timePoint = std::chrono::high_resolution_clock::now();
	double time = std::chrono::duration<double>(timePoint - startTime).count();
	
	baseController.setSetpoint(motionProfile.getPosition_time(time));
	baseController.update();
}

void EctoMagic::setMotionProfileConfig(const MotionProfileConfig &motionProfileConfig) {
	MotionProfileConfig config = motionProfileConfig;
	
	config.initialPosition = baseController.getControllerSource().getPosition();
	config.initialVelocity = baseController.getControllerSource().getVelocity();
	
	config.finalPosition = this->motionProfileConfig.finalPosition;
	
	const auto timePoint = std::chrono::high_resolution_clock::now();
	config.startTime = std::chrono::duration<double>(timePoint - startTime).count();
	
	motionProfile = TrapezoidalMotionProfile(config);
}

const EctoControllerSource &EctoMagic::getControllerSource() const {
	return baseController.getControllerSource();
}

const EctoControllerOutput &EctoMagic::getControllerOutput() const {
	return baseController.getControllerOutput();
}
