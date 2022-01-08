//
// Created by abiel on 11/3/21.
//

#ifndef BOTBUSTERS_REBIRTH_NETWORKTABLEPID_H
#define BOTBUSTERS_REBIRTH_NETWORKTABLEPID_H

#include <functional>
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableInstance.h"

#include "Control/EctoPID/PIDConfig.h"

#include <frc/Notifier.h>
#include <memory>

/*
 * Helper class to calibrate PIDs from network table
 */
class NetworkTablePID {
public:
	NetworkTablePID(const std::string &tableName, PIDConfig defaultValue,
	                const std::function<void(const PIDConfig &config)> &updateFunction);

private:
	std::shared_ptr<nt::NetworkTable> table;
	
	std::function<void(const PIDConfig &config)> updateFunction;
	PIDConfig prevDefinition;
	
	std::unique_ptr<frc::Notifier> notifier;
	const units::millisecond_t updateRate{200};
};


#endif //BOTBUSTERS_REBIRTH_NETWORKTABLEPID_H
