//
// Created by abiel on 10/23/19.
//

#ifndef ECTOCONTROL_EXPONENTIALODOMETRY_H
#define ECTOCONTROL_EXPONENTIALODOMETRY_H

#include <chrono>
#include <Math/DataTypes/RobotPose2D.h>
#include <Math/DataTypes/Twist2D.h>
#include <Control/Odometry/Odometry.h>
#include <Eigen/Core>

class ExponentialOdometry : public Odometry {
public:
	ExponentialOdometry(const RobotPose2D &startingPosition);
	
	ExponentialOdometry();
	
	void updateOdometry_distance(const Twist2D &distance) override;
	
	void updateOdometry_velocity(const Twist2D &velocity, double dt) override;
	
	RobotPose2D getPose() const override;
	
	void resetPosition(const RobotPose2D &pose) override;

private:
	Twist2D lastDistances;
	double lastYaw;
	
	RobotPose2D currentPose;
};


#endif //ECTOCONTROL_EXPONENTIALODOMETRY_H
