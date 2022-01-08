//
// Created by abiel on 8/26/19.
//

#include "Control/Kinematics/Tank/TankMotorValues.h"

TankMotorValues::TankMotorValues(double left, double right) {
	this->left = left;
	this->right = right;
}

TankMotorValues::TankMotorValues() : TankMotorValues(0.0, 0.0) {
	;
}

double TankMotorValues::getHighestVelocity() const {
	return std::max(left, right);
}

void TankMotorValues::clampVelocities(double maxVel) {
	double highestVel = getHighestVelocity();
	
	left = (left / highestVel) * maxVel;
	right = (right / highestVel) * maxVel;
}

void TankMotorValues::normalizeVelocities() {
	clampVelocities(1.0);
}