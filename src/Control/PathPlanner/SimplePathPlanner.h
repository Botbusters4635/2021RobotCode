//
// Created by abiel on 12/25/19.
//

#ifndef ECTOCONTROL_SIMPLEPATHPLANNER_H
#define ECTOCONTROL_SIMPLEPATHPLANNER_H

#include <Math/DataTypes/RobotPose2D.h>
#include "PathPlanner.h"

class SimplePathPlanner : public PathPlanner {
public:
	SimplePathPlanner(const Path &path, double lookaheadDistance, double finishThreshold);
	
	void setNewPath(const Path &newPath) override;
	
	RobotPose2D update(const RobotPose2D &currentPose) override;
	
	double getDistanceToPathCompletion() const override;
	
	bool hasFinished() const override;

private:
	Path currentPath;
	size_t pathSize;
	
	RobotPose2D lastPose;
	
	size_t currentWaypointIndex = 0;
	double lookaheadDistance, finishTreshold;
};


#endif //ECTOCONTROL_SIMPLEPATHPLANNER_H
