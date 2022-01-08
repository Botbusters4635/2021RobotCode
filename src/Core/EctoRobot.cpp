//
// Created by hiram on 28/06/19.
//

#include "EctoRobot.h"
#include <frc/livewindow/LiveWindow.h>
#include "Core/EctoChecklist/ChecklistItem.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <memory>

EctoRobot::EctoRobot(const std::string &robotName) : TimedRobot(units::millisecond_t(EctoRobotConfig::UPDATE_RATE_MS)),
                                                     System(robotName) {
	systemManager = std::make_shared<SystemHandler>("SystemManager");
	inputHandlers = std::make_shared<SystemHandler>("InputHandlers");
	spdlog::set_level(spdlog::level::trace);
	
	systemManager->addSubsystem(managerHandler);
	
	systemManagerTimingPublisher = std::make_shared<TimingDataPublisher>(systemManager, "SystemTimingData");
	inputHandlerTimingPublisher = std::make_shared<TimingDataPublisher>(inputHandlers, "InputHandlerTimingData");
	
	systemManager->addSubsystem(systemManagerTimingPublisher);
	systemManager->addSubsystem(inputHandlerTimingPublisher);
}

void EctoRobot::RobotInit() {
	log->info("RobotInit...");
	frc::LiveWindow::GetInstance()->DisableAllTelemetry();
	
	/**
	 * Motor handler initialization
 	*/
	motorManager.setMotorInfo(getMotorConfig());
	motorManager.init();
	
	/**
	 * PCMManager initialization
	 */
	auto pistonConfig = getPistonConfig();
	if (enablePistonManager) {
		pcmManager.setPCMConfig(pistonConfig);
		pcmManager.init();
	}
	
	robotInit();
	systemManager->robotInit();
	inputHandlers->robotInit();
}


void EctoRobot::RobotPeriodic() {
	systemManager->robotUpdate();
	robotUpdate();
}

//TODO Run command handler here
void EctoRobot::AutonomousInit() {
	commandScheduler.CancelAll();
	commandScheduler.Enable();
	log->info("AutonomousInit... ");
	autoInit();
}

void EctoRobot::AutonomousPeriodic() {
	frc::SmartDashboard::PutData(&commandScheduler);
	commandScheduler.Run();
	autoUpdate();
}

void EctoRobot::TeleopInit() {
	commandScheduler.Disable();
	commandScheduler.CancelAll();
	log->info("TeleopInit...");
	teleopInit();
}

void EctoRobot::TeleopPeriodic() {
	inputHandlers->robotUpdate();
	teleopUpdate();
}

void EctoRobot::DisabledInit() {
	log->info("DisabledInit...");
	systemManager->disabledInit();
	this->disabledInit();
}

void EctoRobot::DisabledPeriodic() {
	systemManager->disabledUpdate();
	this->disabledUpdate();
}

//TODO Add test runner
void EctoRobot::TestInit() {
	commandScheduler.Disable();
	commandScheduler.CancelAll();
	commandScheduler.Enable();
	
	frc2::Command *item = new ChecklistTest();
	commandScheduler.Schedule(item);
	log->info("TestInit...");
	
	this->testInit();
}

void EctoRobot::TestPeriodic() {
	commandScheduler.Run();
	this->testUpdate();
}


