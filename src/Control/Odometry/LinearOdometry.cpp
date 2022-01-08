//
// Created by abiel on 10/23/19.
//

#include "Control/Odometry/LinearOdometry.h"
#include <iostream>

LinearOdometry::LinearOdometry(const RobotPose2D &startingPosition) {
	currentPose = startingPosition;
}

LinearOdometry::LinearOdometry() : LinearOdometry(RobotPose2D(0, 0, 0)) {
	;
}

void LinearOdometry::updateOdometry_distance(const Twist2D &distance) {
	Twist2D deltaDistance = distance - lastDistance;
	//Use given yaw (not delta)
	deltaDistance.setDtheta(distance.getDtheta());
	
	updateOdometry_velocity(deltaDistance, 1);
	
	lastDistance = distance;
}

void LinearOdometry::updateOdometry_velocity(const Twist2D &velocity, double dt) {
	const Twist2D distance = velocity * dt;
	
	const double cosAngle = std::cos(velocity.getDtheta());
	const double sinAngle = std::sin(velocity.getDtheta());
	const double xRotated = cosAngle * distance.getDx() + sinAngle * distance.getDy();
	const double yRotated = cosAngle * distance.getDy() - sinAngle * distance.getDx();
	
	currentPose.setX(currentPose.getX() + xRotated);
	currentPose.setY(currentPose.getY() - yRotated);
	currentPose.setHeading(Rotation2D(velocity.getDtheta()));
}

RobotPose2D LinearOdometry::getPose() const {
	return currentPose;
}

void LinearOdometry::resetPosition(const RobotPose2D &pose) {
	currentPose = pose;
}