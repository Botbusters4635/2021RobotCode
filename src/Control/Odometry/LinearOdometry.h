//
// Created by abiel on 10/23/19.
//

#ifndef ECTOCONTROL_LINEARODOMETRY_H
#define ECTOCONTROL_LINEARODOMETRY_H

#include "Math/DataTypes/RobotPose2D.h"
#include "Math/DataTypes/Twist2D.h"
#include "Control/Odometry/Odometry.h"

/**
 * Calculates odometry with linear interpolation
 */
class LinearOdometry : public Odometry {
public:
	LinearOdometry(const RobotPose2D &startingPosition);
	
	LinearOdometry();
	
	void updateOdometry_distance(const Twist2D &distance) override;
	
	void updateOdometry_velocity(const Twist2D &velocity, double dt) override;
	
	RobotPose2D getPose() const override;
	
	void resetPosition(const RobotPose2D &pose) override;

private:
	Twist2D lastDistance;
	
	RobotPose2D currentPose;
};


#endif //ECTOCONTROL_LINEARODOMETRY_H
