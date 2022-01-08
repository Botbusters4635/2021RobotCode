//
// Created by abiel on 11/16/21.
//

#include <frc/kinematics/ChassisSpeeds.h>
#include "SimulationRobot.h"
#include <tuple>

SimulationRobot::SimulationRobot() : EctoRobot("SimulationRobot") {
	;
}

void SimulationRobot::robotInit() {
	swerve = std::make_shared<EctoSwerve>(EctoSwerveConfig());
	swerveInputHandler = std::make_shared<EctoSwerveInputHandler>(*swerve);
	
	systemManager->addSubsystem(swerve);
	inputHandlers->addSubsystem(swerveInputHandler);
}

void SimulationRobot::teleopInit() {
	lastTime = frc::Timer::GetFPGATimestamp();
}

void SimulationRobot::teleopUpdate() {
	std::vector<std::tuple<double, double, double>> setpoints = {
			std::make_tuple(1, 0, 0),
			std::make_tuple(-1, 0, M_PI_2),
			std::make_tuple(1, 0, M_PI),
			std::make_tuple(-1, 0, -M_PI_2),
			std::make_tuple(1, 0, -M_PI)};
	bool doPrint = false;
	if (frc::Timer::GetFPGATimestamp() - lastTime > 5) {
		i++;
		if (i >= setpoints.size()) i = 0;
		lastTime = frc::Timer::GetFPGATimestamp();
		log->info("Switching setpoints!");
		doPrint = true;
	}
	double x, y, theta;
	std::tie(x, y, theta) = setpoints[i];
	if (doPrint) log->info("{},{},{}", x, y, theta);
	frc::SmartDashboard::PutNumber("In/X", x);
	frc::SmartDashboard::PutNumber("In/Y", y);
	frc::SmartDashboard::PutNumber("In/Theta", theta);
	
	swerve->setYaw(theta);
	
	auto chassisSpeeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(units::meters_per_second_t(x),
	                                                                 units::meters_per_second_t(y),
	                                                                 units::radians_per_second_t(0),
	                                                                 frc::Rotation2d(
			                                                                 units::radian_t(swerve->getYaw())));
	
	frc::SmartDashboard::PutNumber("Out/X", chassisSpeeds.vx.value());
	frc::SmartDashboard::PutNumber("Out/Y", chassisSpeeds.vy.value());
	frc::SmartDashboard::PutNumber("Out/Theta", chassisSpeeds.omega.value());
	//swerve->setVoltage(chassisSpeeds);
}