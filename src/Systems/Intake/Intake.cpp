//
// Created by Neil Rodriguez Murillo on 10/8/21.
//

#include "Intake.h"

Intake::Intake(const IntakeConfig &config) : System("Intake") {
	this->intakeConfig = config;
	
	config.motor->setOpenLoopRampRate(config.rampRate);
	config.motor->setMotorCurrentLimit(config.currentLimit);
	config.motor->enableCurrentLimit(true);
	config.motor->invertMotor(config.invertMotor);
}

void Intake::set(double pct, bool extended) {
//    auto vol = xRightJoy.get();
//    auto state = leftShoulder.get();
	intakeConfig.motor->set(pct, MotorControlMode::Percent);
	PCMManager::set(intakeConfig.piston, extended);
}


