//
// Created by alberto on 15/08/19.
//

#include "Control/Kinematics/Swerve/SwerveKinematics.h"
#include <iostream>
#include "Math/EctoMath.h"

SwerveKinematics::SwerveKinematics(double length, double width) {
	this->length = length;
	this->width = width;
	
	radius = std::hypot(length, width);
	lengthComponent = length / radius;
	widthComponent = width / radius;
	lastRunTime = std::chrono::high_resolution_clock::now();
	
	//Relative from center (0,0)
	inverseKinematics <<
	                  1, 0, (width / 2.0), //Top Left
			0, 1, (length / 2.0),
			
			1, 0, (-width / 2.0), //Top right
			0, 1, (length / 2.0),
			
			1, 0, (width / 2.0), //Back left
			0, 1, -(length / 2.0),
			
			1, 0, (-width / 2.0), //Back right
			0, 1, -(length / 2.0);
	
	previousCenterOfRotation = Point2D(0, 0);
	
	forwardKinematics = SwerveKinematics::pseudoinverse(inverseKinematics);
}

SwerveState SwerveKinematics::OLDcalculateInverseKinematics(const Twist2D &targetVelocity,
                                                            const SwerveState &currentState) {
	double dt = std::chrono::duration_cast<std::chrono::duration<double>>(
			std::chrono::high_resolution_clock::now() - lastRunTime).count();
	SwerveState output;
	
	double forward = targetVelocity.getDx();
	double strafe = targetVelocity.getDy();
	double rotation = targetVelocity.getDtheta();
	
	double a = strafe - rotation * lengthComponent;
	double b = strafe + rotation * lengthComponent;
	double c = forward - rotation * widthComponent;
	double d = forward + rotation * widthComponent;
	
	output.topLeft.wheelVelocity = std::hypot(b, c);
	output.topRight.wheelVelocity = std::hypot(b, d);
	output.backRight.wheelVelocity = std::hypot(a, d);
	output.backLeft.wheelVelocity = std::hypot(a, c);
	
	if (targetVelocity.getDtheta() == 0 && (targetVelocity.getDx() == 0 && targetVelocity.getDy() == 0)) {
		output.topLeft.wheelAngle = currentState.topLeft.wheelAngle;
		output.topRight.wheelAngle = currentState.topRight.wheelAngle;
		output.backRight.wheelAngle = currentState.backRight.wheelAngle;
		output.backLeft.wheelAngle = currentState.backLeft.wheelAngle;
	} else {
		output.topLeft.wheelAngle = std::atan2(b, c);
		output.topRight.wheelAngle = std::atan2(b, d);
		output.backRight.wheelAngle = std::atan2(a, d);
		output.backLeft.wheelAngle = std::atan2(a, c);
	}

//    output.normalizeVelocities();
	
	output = SwerveState::optimizeValues(currentState, output, lastState, lastSet, dt);
	lastSet = output;
	
	lastState = currentState;
	lastRunTime = std::chrono::high_resolution_clock::now();
	
	return output;
}

SwerveState SwerveKinematics::calculateInverseKinematics(const Twist2D &targetVelocity, const SwerveState &currentState,
                                                         const Point2D &centerOfRotation) {
	double dt = std::chrono::duration_cast<std::chrono::duration<double>>(
			std::chrono::high_resolution_clock::now() - lastRunTime).count();
	
	SwerveState output;
	
	if (centerOfRotation != previousCenterOfRotation) {
		const double x = centerOfRotation.getX();
		const double y = centerOfRotation.getY();
		
		inverseKinematics <<
		                  1, 0, (width / 2.0) + y, //Top Left
				0, 1, (length / 2.0) + x,
				
				1, 0, (-width / 2.0) + y, //Top right
				0, 1, (length / 2.0) + x,
				
				1, 0, (width / 2.0) + y, //Back left
				0, 1, -(length / 2.0) + x,
				
				1, 0, (-width / 2.0) + y, //Back right
				0, 1, (-length / 2.0) + x;
		
		forwardKinematics = SwerveKinematics::pseudoinverse(inverseKinematics);
		
		previousCenterOfRotation = centerOfRotation;
	}
	
	Eigen::Matrix<double, 3, 1> velocityMatrix;
	velocityMatrix << -targetVelocity.getDx(), targetVelocity.getDy(), targetVelocity.getDtheta();
	
	Eigen::Matrix<double, 8, 1> moduleStatesMatrix;
	moduleStatesMatrix = inverseKinematics * velocityMatrix;
	
	double x, y;
	
	//Top Left
	x = moduleStatesMatrix(0, 0);
	y = moduleStatesMatrix(1, 0);
	calculateWheelValue(x, y, output.topLeft, lastSet.topLeft);
	
	//Top Right
	x = moduleStatesMatrix(2, 0);
	y = moduleStatesMatrix(3, 0);
	calculateWheelValue(x, y, output.topRight, lastSet.topRight);
	
	//Back Left
	x = moduleStatesMatrix(4, 0);
	y = moduleStatesMatrix(5, 0);
	calculateWheelValue(x, y, output.backLeft, lastSet.backLeft);
	
	//Back right
	x = moduleStatesMatrix(6, 0);
	y = moduleStatesMatrix(7, 0);
	calculateWheelValue(x, y, output.backRight, lastSet.backRight);
	
	//output.normalizeVelocities();
	
	output = SwerveState::optimizeValues(currentState, output, lastState, lastSet, dt);
	output.roundToEpsilon();
	
	lastSet = output;
	
	lastState = currentState;
	lastRunTime = std::chrono::high_resolution_clock::now();
	
	return output;
}

SwerveState SwerveKinematics::calculateInverseKinematics(const Twist2D &targetVelocity,
                                                         const SwerveState &currentState) {
	return calculateInverseKinematics(targetVelocity, currentState, Point2D(0, 0));
}

void SwerveKinematics::calculateWheelValue(double x, double y, SwerveWheel &wheel, const SwerveWheel &lastValue) const {
	wheel.wheelVelocity = std::hypot(x, y);
	if (wheel.wheelVelocity > 1e-9) {
		wheel.wheelAngle = std::atan2(y / wheel.wheelVelocity, x / wheel.wheelVelocity);
	} else {
		wheel.wheelAngle = std::atan2(0.0, 1.0);
	}
}

Twist2D SwerveKinematics::calculateForwardKinematics(const SwerveState &currentValues) const {
	Eigen::Matrix<double, 8, 1> moduleStatesMatrix;
	moduleStatesMatrix <<
	                   //Top left
	                   currentValues.topLeft.wheelVelocity * std::cos(currentValues.topLeft.wheelAngle),
			currentValues.topLeft.wheelVelocity * std::sin(currentValues.topLeft.wheelAngle),
			
			//Top right
			currentValues.topRight.wheelVelocity * std::cos(currentValues.topRight.wheelAngle),
			currentValues.topRight.wheelVelocity * std::sin(currentValues.topRight.wheelAngle),
			
			//Back left
			currentValues.backLeft.wheelVelocity * std::cos(currentValues.backLeft.wheelAngle),
			currentValues.backLeft.wheelVelocity * std::sin(currentValues.backLeft.wheelAngle),
			
			//Back right
			currentValues.backRight.wheelVelocity * std::cos(currentValues.backRight.wheelAngle),
			currentValues.backRight.wheelVelocity * std::sin(currentValues.backRight.wheelAngle);
	
	Eigen::Matrix<double, 3, 1> speedsMatrix = forwardKinematics * moduleStatesMatrix;
	
	Twist2D output(-speedsMatrix(0, 0), speedsMatrix(1, 0), speedsMatrix(2, 0));
	output.roundToEpsilon();
	
	return output;
}
