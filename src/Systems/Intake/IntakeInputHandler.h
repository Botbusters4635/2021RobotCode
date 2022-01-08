//
// Created by Neil Rodriguez Murillo on 10/8/21.
//

#ifndef BOTBUSTERS_REBIRTH_INTAKEINPUTHANDLER_H
#define BOTBUSTERS_REBIRTH_INTAKEINPUTHANDLER_H

#include <Core/EctoInput/InputManager.h>
#include "Systems/Intake/Intake.h"

#include <Core/EctoInput/Axis/JoystickAxisExpo.h>


class IntakeInputHandler : public System {
public:
	IntakeInputHandler(const std::shared_ptr<Intake> &intake) : System("IntakeInputHandler") {
		this->intake = intake;
	}
	
	void robotInit();
	
	void robotUpdate();

private:
	//CONTROL INPUTS left and right shoulder button are switched
	EctoButton leftShoulder;
	JoystickAxisExpo xRightJoy{0.1, 0.45};
	EctoButton rightShoulder;
	//CONTROL INPUT NAMES
	static constexpr auto intakeJoy = "leftY";
	static constexpr auto intakePistonButton = "rightBumper";
	static constexpr auto intakePistonAndSpitButton = "leftBumper";
	
	
	//INSTANCES
	InputManager &input = InputManager::getInstance();
	std::shared_ptr<Intake> intake;
	IntakeConfig config;
	
};


#endif //BOTBUSTERS_REBIRTH_INTAKEINPUTHANDLER_H
