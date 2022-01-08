//
// Created by Abiel on 8/26/19.
//

#include "Control/Kinematics/Swerve/SwerveWheel.h"
#include <cmath>
#include "Math/EctoMath.h"

void SwerveWheel::invertValue() {
	wheelVelocity *= -1.0;
	wheelAngle += std::copysign(M_PI, wheelAngle);
	wheelAngle = (wheelAngle > M_PI ? wheelAngle - 2 * M_PI : wheelAngle);
	wheelAngle = (wheelAngle < -M_PI ? wheelAngle + 2 * M_PI : wheelAngle);
}

SwerveWheel SwerveWheel::invertValue_copy(const SwerveWheel &value) {
	SwerveWheel output(value);
	output.invertValue();
	return output;
}

double SwerveWheel::getAngleBetween(const SwerveWheel &current, const SwerveWheel &target) {
	return EctoMath::wrapAngle(target.wheelAngle - current.wheelAngle);
}

bool SwerveWheel::operator==(const SwerveWheel &rhs) const {
	return wheelVelocity == rhs.wheelVelocity and
	       wheelAngle == rhs.wheelAngle and
	       wheelAngularVelocity == rhs.wheelAngularVelocity;
}

void SwerveWheel::roundToEpsilon() {
	if (std::abs(wheelVelocity) < epsilon) {
		wheelVelocity = 0.0;
	}
	
	if (std::abs(wheelAngle) < epsilon) {
		wheelAngle = 0.0;
	}
	
	if (std::abs(wheelAngularVelocity) < epsilon) {
		wheelAngularVelocity = 0.0;
	}
}

SwerveWheel SwerveWheel::roundToEpsilon_copy(const SwerveWheel &wheel) {
	SwerveWheel output(wheel);
	output.roundToEpsilon();
	return output;
}

std::ostream &operator<<(std::ostream &os, const SwerveWheel &wheel) {
	os << "Wheel Velocity: " << wheel.wheelVelocity << " Wheel Angle: " << wheel.wheelAngle
	   << " Wheel Angular Velocity: " << wheel.wheelAngularVelocity;
	return os;
}