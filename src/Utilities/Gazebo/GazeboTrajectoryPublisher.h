//
// Created by abiel on 7/16/20.
//

#ifndef BOTBUSTERSREBIRTH_GAZEBOTRAJECTORYPUBLISHER_H
#define BOTBUSTERSREBIRTH_GAZEBOTRAJECTORYPUBLISHER_H

#include <networktables/NetworkTableEntry.h>
#include <frc/trajectory/Trajectory.h>

namespace GazeboTrajectoryPublisher {
	/**
	 * Serializes an frc::Trajectory into a string
	 */
	std::string serializeTrajectory(const frc::Trajectory &trajectory);
	
	/**
	 * Serializes an frc::Trajectory and sends it through NT
	 */
	void sendTrajectory(const frc::Trajectory &trajectory, nt::NetworkTableEntry &entry);
}


#endif //BOTBUSTERSREBIRTH_GAZEBOTRAJECTORYPUBLISHER_H
