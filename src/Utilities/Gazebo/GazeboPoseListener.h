//
// Created by abiel on 7/17/20.
//

#ifndef BOTBUSTERSREBIRTH_GAZEBOPOSELISTENER_H
#define BOTBUSTERSREBIRTH_GAZEBOPOSELISTENER_H

#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include <Math/DataTypes/RobotPose2D.h>

/**
 * Listens to a new pose sent by gazebo through nt
 * runs synchronously in order to avoid any race conditions
 */
class GazeboPoseListener {
public:
	explicit GazeboPoseListener(const std::string &ntEntryName);
	
	explicit GazeboPoseListener(const nt::NetworkTableEntry &entry);
	
	bool hasNewValue() const;
	
	RobotPose2D getPose() const;

private:
	nt::NetworkTableInstance ntInstance = nt::NetworkTableInstance::GetDefault();
	nt::NetworkTableEntry entry;
	
	mutable std::string previouslyReadValue;
};


#endif //BOTBUSTERSREBIRTH_GAZEBOPOSELISTENER_H
