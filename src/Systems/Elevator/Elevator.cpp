//
// Created by Abiel on 9/9/19.
//

#include "Elevator.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <networktables/NetworkTableInstance.h>
#include <iostream>

Elevator::Elevator(const std::shared_ptr<EctoMotor> &motor) : System("Elevator") {
	motor->enableBrakingOnIdle(true);
	this->motor = motor;
}

void Elevator::climb(double output) {
	motor->set(output, MotorControlMode::Percent);
}
