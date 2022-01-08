//
// Created by abiel on 10/23/19.
//

#include "Control/Odometry/ExponentialOdometry.h"
#include "Math/EctoMath.h"
#include "Math/DataTypes/RobotPose2D.h"

ExponentialOdometry::ExponentialOdometry(const RobotPose2D &startingPosition) {
	currentPose = startingPosition;
}

ExponentialOdometry::ExponentialOdometry() : ExponentialOdometry(RobotPose2D(0, 0, 0)) {
	;
}

#include <iostream>

void ExponentialOdometry::updateOdometry_distance(const Twist2D &distance) {
	Twist2D deltaDistance = distance - lastDistances;
	//Use given yaw (not delta)
	deltaDistance.setDtheta(distance.getDtheta());
	
	updateOdometry_velocity(deltaDistance, 1);
	
	lastDistances = distance;
}

void ExponentialOdometry::updateOdometry_velocity(const Twist2D &velocity, double dt) {
	const double yaw = velocity.getDtheta();
	const double yawCos = std::cos(yaw);
	const double yawSin = std::sin(yaw);
	
	const double angularVelocity = (yaw - lastYaw) / dt;
	
	auto newVelocity = velocity;
	newVelocity *= dt;
	newVelocity.setDtheta(angularVelocity);
	
	const double temp = newVelocity.getDx() * yawCos + newVelocity.getDy() * yawSin;
	newVelocity.setDy(newVelocity.getDy() * yawCos - newVelocity.getDx() * yawSin);
	newVelocity.setDx(temp);
	
	auto expVelocity = RobotPose2D::exp(newVelocity);
	expVelocity.setY(-expVelocity.getY());
	
	currentPose += expVelocity;
	currentPose.setHeading(Rotation2D(yaw));
	
	lastYaw = yaw;
}

RobotPose2D ExponentialOdometry::getPose() const {
	return currentPose;
}

void ExponentialOdometry::resetPosition(const RobotPose2D &pose) {
	currentPose = pose;
}