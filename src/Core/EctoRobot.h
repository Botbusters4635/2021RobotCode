//
// Created by hiram on 28/06/19.
//

#ifndef  BOTBUSTERS_REBIRTH_ECTOROBOT_H
#define BOTBUSTERS_REBIRTH_ECTOROBOT_H

#include <frc2/command/CommandScheduler.h>
#include <frc/TimedRobot.h>
#include <map>
#include <Core/EctoModule/System.h>
#include <Core/EctoModule/SystemHandler.h>
#include <spdlog/spdlog.h>
#include <Core/MotorHandler/MotorManager.h>
#include <Core/EctoInput/InputManager.h>
#include <Core/EctoModule/ManagerHandler.h>

#include "Systems/TimingDataPublisher/TimingDataPublisher.h"
#include "Core/EctoRobotConfig.h"

#include "Core/PCM/PCMManager.h"
#include "frc/smartdashboard/SmartDashboard.h"


class EctoRobot : public frc::TimedRobot, public System {
public:
	explicit EctoRobot(const std::string &robotName);
	
	std::shared_ptr<SystemHandler> systemManager;
	std::shared_ptr<SystemHandler> inputHandlers;
	
	static constexpr bool updateInputsInAuto = false;
	
	PCMManager &pcmManager = PCMManager::getInstance();
	MotorManager &motorManager = MotorManager::getInstance();
	InputManager &inputManager = InputManager::getInstance();
	
	virtual void autoInit() { ; };
	
	virtual void autoUpdate() { ; };
	
	virtual void teleopInit() { ; };
	
	virtual void teleopUpdate() { ; };
	
	virtual void testInit() { ; };
	
	virtual void testUpdate() { ; };

protected:
	/**
	 * set this in order to define which motors are @ which ids
	 * @return
	 */
	virtual std::list<MotorInfo> getMotorConfig() = 0;
	
	virtual std::list<PistonInfo> getPistonConfig() {
		enablePistonManager = false;
		return {};
	}
	
	frc2::CommandScheduler &commandScheduler = frc2::CommandScheduler::GetInstance();
private:
	std::shared_ptr<TimingDataPublisher> systemManagerTimingPublisher, inputHandlerTimingPublisher;
	ManagerHandler &managerHandler = ManagerHandler::getInstance();
	
	//frc::Sendable commandSchedulerSendable = commandScheduler.
	
	bool enablePistonManager = true;
	
	void RobotInit() override final;
	
	void RobotPeriodic() override final;
	
	void TestInit() override final;
	
	void TestPeriodic() override final;
	
	void DisabledInit() override final;
	
	void DisabledPeriodic() override final;
	
	void TeleopInit() override final;
	
	void TeleopPeriodic() override final;
	
	void AutonomousInit() override final;
	
	void AutonomousPeriodic() override final;
};


#endif //BOTBUSTERS_REBIRTH_ECTOROBOT_H
