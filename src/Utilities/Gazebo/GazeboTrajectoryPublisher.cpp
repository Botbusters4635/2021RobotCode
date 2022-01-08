//
// Created by abiel on 7/16/20.
//

#include "GazeboTrajectoryPublisher.h"
#include <sstream>

std::string GazeboTrajectoryPublisher::serializeTrajectory(const frc::Trajectory &trajectory) {
	std::stringstream buf;
	buf << trajectory.States().size() << ',';
	for (const auto &state: trajectory.States()) {
		buf << state.pose.Translation().X().value() << ',' << state.pose.Translation().Y().value() << ',';
		buf << state.pose.Rotation().Radians().value();
		buf << ',';
	}
	
	return buf.str();
}

void GazeboTrajectoryPublisher::sendTrajectory(const frc::Trajectory &trajectory, nt::NetworkTableEntry &entry) {
	std::string serializedTraj = GazeboTrajectoryPublisher::serializeTrajectory(trajectory);
	entry.SetString(serializedTraj);
}