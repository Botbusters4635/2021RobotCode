//
// Created by abiel on 12/25/19.
//

#include "Control/Path/Path.h"
#include <iostream>

Path::Path() {
	;
}

Path::Path(const std::vector<RobotPose2D> &poses) {
	this->poses = poses;
}

const std::vector<RobotPose2D> &Path::getPoses() const {
	return poses;
}

std::ostream &operator<<(std::ostream &os, const Path &value) {
	for (const auto &pose: value.getPoses()) {
		os << pose << std::endl;
	}
	
	return os;
}