//
// Created by abiel on 8/26/19.
//

#include "Control/Kinematics/Tank/TankKinematics.h"

TankKinematics::TankKinematics(double length, double width) {
	this->length = length;
	this->width = width;
}

TankMotorValues TankKinematics::calculateKinematics(const Twist2D &targetVelocity) const {
	//TODO Implement this
	throw std::runtime_error("Not implemented yet!");
}

Twist2D TankKinematics::calculateInverseKinematics(const TankMotorValues &currentValues) const {
	//TODO Implement this
	throw std::runtime_error("Not implemented yet!");
}