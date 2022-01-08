//
// Created by abiel on 11/16/21.
//

#ifndef BOTBUSTERS_REBIRTH_SIMULATIONROBOT_H
#define BOTBUSTERS_REBIRTH_SIMULATIONROBOT_H

#include <networktables/NetworkTableInstance.h>
#include <Core/EctoInput/Axis/JoystickAxisExpo.h>
#include <Systems/EctoSwerve/EctoSwerve.h>
#include <Systems/EctoSwerve/EctoSwerveInputHandler.h>
#include "Core/EctoRobot.h"

class SimulationRobot : public EctoRobot {
public:
	SimulationRobot();
	
	void robotInit() override;
	
	void teleopInit() override;
	
	void teleopUpdate();

protected:
	std::list<MotorInfo> getMotorConfig() override {
		return {{EctoMotorType::Simulated, "front_left_wheel",  1},
		        {EctoMotorType::Simulated, "front_right_wheel", 3},
		        {EctoMotorType::Simulated, "back_left_wheel",   7},
		        {EctoMotorType::Simulated, "back_right_wheel",  6},
		
		        {EctoMotorType::Simulated, "front_left_steer",  2},
		        {EctoMotorType::Simulated, "front_right_steer", 4},
		        {EctoMotorType::Simulated, "back_left_steer",   8},
		        {EctoMotorType::Simulated, "back_right_steer",  5},
		
		        {EctoMotorType::Simulated, "shooter_wheel1",    9},
		        {EctoMotorType::Simulated, "shooter_wheel2",    10}};
	}

private:
	double lastTime;
	int i{0};
	
	std::shared_ptr<EctoSwerve> swerve;
	std::shared_ptr<EctoSwerveInputHandler> swerveInputHandler;
};


#endif //BOTBUSTERS_REBIRTH_SIMULATIONROBOT_H
