//
// Created by abiel on 9/10/19.
//

#ifndef BOTBUSTERSREBIRTH_MOTORCHARACTERIZATIONHELPER_H
#define BOTBUSTERSREBIRTH_MOTORCHARACTERIZATIONHELPER_H

#include "Core/MotorHandler/EctoMotor/EctoMotor.h"
#include "Core/EctoModule/System.h"
#include <iostream>
#include <fstream>
#include <frc/smartdashboard/SmartDashboard.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include <frc/RobotController.h>
#include <frc/Timer.h>
#include <fmt/format.h>

class MotorCharacterizationHelper : public System {
public:
	MotorCharacterizationHelper(const std::string &name, const std::string &fileName,
	                            const std::shared_ptr<EctoMotor> &motor);
	
	void robotInit() override;
	
	void robotUpdate() override;

private:
	double startTime = 0;
	const double timeToRunFor = 30.0;
	const double timeToWaitAfterRuns = 16.0;
	
	double timesRan = 0;
	bool waitingToSlowDown = false;
	
	const double timesToRun = 1;
	
	double lastRunTime = 0;
	double lastVelocity = 0;
	
	std::string fileName;
	std::shared_ptr<EctoMotor> motor;
	
	std::ofstream file;
	
	nt::NetworkTableEntry autoSpeedEntry =
			nt::NetworkTableInstance::GetDefault().GetEntry("/MotorCharacterizationHelper/autospeed");
};


#endif //BOTBUSTERSREBIRTH_MOTORCHARACTERIZATIONHELPER_H
