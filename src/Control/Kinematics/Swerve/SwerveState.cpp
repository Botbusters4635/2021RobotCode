//
// Created by alberto on 15/08/19.
//

#include "Control/Kinematics/Swerve/SwerveState.h"
#include <iostream>

SwerveState::SwerveState(const SwerveWheel &topLeft, const SwerveWheel &topRight, const SwerveWheel &backLeft,
                         const SwerveWheel &backRight) {
	this->topLeft = topLeft;
	this->topRight = topRight;
	
	this->backLeft = backLeft;
	this->backRight = backRight;
}

SwerveState::SwerveState(const std::array<frc::SwerveModuleState, 4> &states) {
	for (size_t i = 0; i < wheels.size(); i++) {
		auto &wheel(wheels[i]);
		wheel->wheelVelocity = states[i].speed.value();
		wheel->wheelAngle = states[i].angle.Radians().value();
		wheel->wheelAngularVelocity = 0;
	}
}

SwerveState::SwerveState() {
	;
}

wpi::array<frc::SwerveModuleState, 4> SwerveState::toWPI() const {
	std::array<frc::SwerveModuleState, 4> states;
	for (size_t i = 0; i < wheels.size(); i++) {
		const auto &wheel(wheels[i]);
		states[i].speed = units::velocity::meters_per_second_t(wheel->wheelVelocity);
		states[i].angle = frc::Rotation2d(units::angle::radian_t(wheel->wheelAngle));
	}
	
	return {states};
}

double SwerveState::getAngleWeight(const SwerveWheel &currentState,
                                   const SwerveWheel &targetState) {
	return std::abs(SwerveWheel::getAngleBetween(currentState, targetState) / M_PI);
}

double SwerveState::getVelocityWeight(const SwerveWheel &currentState,
                                      const SwerveWheel &targetState) {
	return std::abs(targetState.wheelVelocity - currentState.wheelVelocity) / 2.0;
}

double SwerveState::getAngleAccelerationWeight(const SwerveWheel &currentState,
                                               const SwerveWheel &targetState, const SwerveWheel &lastState,
                                               double dt) {
	double acceleration = (lastState.wheelAngularVelocity - currentState.wheelAngularVelocity) / dt;
	double angleDifference = SwerveWheel::getAngleBetween(currentState, targetState);
	
	if (angleDifference > 0) {
		return acceleration < 0 ? 1 : 0;
	} else if (angleDifference < 0) {
		return acceleration > 0 ? 1 : 0;
	} else {
		//0
		return 0;
	}
}

double SwerveState::calculateWeight(const SwerveWheel &currentState, const SwerveWheel &targetState,
                                    const SwerveWheel &lastState, double dt) {
	return getAngleWeight(currentState, targetState) * angleWeightValue +
	       getVelocityWeight(currentState, targetState) * velocityWeightValue +
	       getAngleAccelerationWeight(currentState, targetState, lastState, dt) * 0.666;
}

//TODO Refactor SwerveState and remove unused parameters
SwerveWheel SwerveState::optimizeWheelValue(const SwerveWheel &currentState, const SwerveWheel &targetState,
                                            const SwerveWheel &lastState, const SwerveWheel &lastSet, double dt) {
	SwerveWheel invertedState = SwerveWheel::invertValue_copy(targetState);
	double targetWeight = getAngleWeight(lastSet, targetState);
	double invertedWeight = getAngleWeight(lastSet, invertedState);
	
	if (targetWeight > invertedWeight)
		return invertedState;
	else
		return targetState;
}

SwerveState SwerveState::optimizeValues_copy(const SwerveState &currentState, const SwerveState &targetState,
                                             const SwerveState &lastState, const SwerveState &lastSet, double dt) {
	SwerveState output;
	output.topLeft = optimizeWheelValue(currentState.topLeft, targetState.topLeft, lastState.topLeft, lastSet.topLeft,
	                                    dt);
	output.topRight = optimizeWheelValue(currentState.topRight, targetState.topRight, lastState.topRight,
	                                     lastSet.topRight, dt);
	
	output.backLeft = optimizeWheelValue(currentState.backLeft, targetState.backLeft, lastState.backLeft,
	                                     lastSet.backLeft, dt);
	output.backRight = optimizeWheelValue(currentState.backRight, targetState.backRight, lastState.backRight,
	                                      lastSet.backRight, dt);
	return output;
}

SwerveState SwerveState::optimizeValues(const SwerveState &currentState, const SwerveState &targetState,
                                        const SwerveState &lastState, const SwerveState &lastSet, double dt) {
	return SwerveState::optimizeValues_copy(currentState, targetState, lastState, lastSet, dt);
}

bool SwerveState::operator==(const SwerveState &rhs) {
	return topLeft == rhs.topLeft and
	       topRight == rhs.topRight and
	       backLeft == rhs.backLeft and
	       backRight == rhs.backRight;
}

void SwerveState::roundToEpsilon() {
	topLeft.roundToEpsilon();
	topRight.roundToEpsilon();
	
	backLeft.roundToEpsilon();
	backRight.roundToEpsilon();
}

std::ostream &operator<<(std::ostream &os, const SwerveState &value) {
	os << "Top Left: " << value.topLeft << std::endl;
	os << "Top Right: " << value.topRight << std::endl;
	os << "Back Left: " << value.backLeft << std::endl;
	os << "Back Right: " << value.backRight;
	return os;
}