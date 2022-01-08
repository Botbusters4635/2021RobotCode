//
// Created by Neil Rodriguez Murillo on 9/22/21.
//

#include <frc/Timer.h>
#include "EctoTank.h"


EctoTank::EctoTank(const EctoTankConfig &config) : System("Chassis-Tank") {
	this->tankConfig = config;
}

void EctoTank::robotInit() {
	previousTime = frc::Timer::GetFPGATimestamp();
}

void EctoTank::robotUpdate() {
	;
}

void EctoTank::setTargetVelocity(double left, double right) {
	tankConfig.rightMotors[0]->set(right, MotorControlMode::Velocity);
	tankConfig.leftMotors[0]->set(left, MotorControlMode::Velocity);
	
}

void EctoTank::setPercentage(double left, double right) {
	
	tankConfig.rightMotors[0]->set(right, MotorControlMode::Percent);
	tankConfig.leftMotors[0]->set(left, MotorControlMode::Percent);
	
	
}
