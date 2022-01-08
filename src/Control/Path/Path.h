//
// Created by abiel on 12/25/19.
//

#ifndef ECTOCONTROL_PATH_H
#define ECTOCONTROL_PATH_H

#include <vector>
#include <ostream>
#include "Math/DataTypes/RobotPose2DCurvature.h"

class Path {
public:
	Path();
	
	Path(const std::vector<RobotPose2D> &poses);
	
	[[nodiscard]] const std::vector<RobotPose2D> &getPoses() const;
	
	RobotPose2D &operator[](size_t i) {
		return poses[i];
	}
	
	const RobotPose2D &at(size_t i) {
		return poses.at(i);
	}
	
	friend std::ostream &operator<<(std::ostream &os, const Path &value);

private:
	std::vector<RobotPose2D> poses;
};

#endif //ECTOCONTROL_PATH_H
