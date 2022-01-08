//
// Created by abiel on 12/25/19.
//

#ifndef ECTOCONTROL_PATHPLANNER_H
#define ECTOCONTROL_PATHPLANNER_H

#include "Math/DataTypes/RobotPose2D.h"
#include "Control/Path/Path.h"

class PathPlanner {
public:
	/**
	 * Sets a new target path
	 * @param newPath
	 */
	virtual void setNewPath(const Path &newPath) = 0;
	
	virtual RobotPose2D update(const RobotPose2D &currentPose) = 0;
	
	//How close is the follower to completing the path
	virtual double getDistanceToPathCompletion() const = 0;
	
	virtual bool hasFinished() const = 0;
};

#endif //ECTOCONTROL_PATHPLANNER_H
