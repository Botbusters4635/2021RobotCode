//
// Created by Abiel on 10/30/17.
//

#include "Math/EctoMath.h"
#include "Math/DataTypes/Rotation2D.h"

//TODO Find out what all of these functions do, and comment them

double EctoMath::radiansToDegrees(double radians) {
	return (radians / M_PI) * 180.0;
}

double EctoMath::degreesToRadians(double degrees) {
	return (degrees / 180.0) * M_PI;
}

double EctoMath::addTwoAngles(double first, double second) {
	return wrapAngle(first + second);
}

double EctoMath::wrapAngle(double angle) {
	angle = std::copysign(std::fmod(angle, 2 * M_PI), angle);
	if (angle > M_PI) {
		angle -= 2 * M_PI;
	} else if (angle < -M_PI) {
		angle += 2 * M_PI;
	}
	return angle;
}

double EctoMath::shortestAngleTurn(double current, double target) {
	double angleToTurn = target - current;
	
	if (std::abs(angleToTurn) > M_PI) {
		if (current > 0) {
			if (angleToTurn > 0) {
				angleToTurn = angleToTurn - M_PI;
			} else {
				angleToTurn = angleToTurn + 2.0 * M_PI;
			}
		} else {
			if (angleToTurn > 0) {
				angleToTurn = angleToTurn - 2.0 * M_PI;
			} else {
				angleToTurn = angleToTurn + M_PI;
			}
		}
	}
	
	return angleToTurn;
}

Rotation2D EctoMath::convertTo360(const Rotation2D &input) {
	double radians = input.getRadians();
	
	return Rotation2D::fromRadians(radians + M_PI);
}

double EctoMath::sinc(double x) {
	if (x == 0)
		return 1;
	
	return std::sin(x) / x;
}

double EctoMath::cosc(double x) {
	if (x == 0)
		return 1;
	
	return std::cos(x) / x;
}