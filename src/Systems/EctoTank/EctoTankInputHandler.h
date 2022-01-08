//
// Created by abiel on 7/15/20.
//

#ifndef BOTBUSTERSREBIRTH_ECTOTANKINPUTHANDLER_H
#define BOTBUSTERSREBIRTH_ECTOTANKINPUTHANDLER_H

#include "EctoTank.h"
#include <Core/EctoModule/System.h>

#include <Core/EctoInput/InputManager.h>
#include <Core/EctoInput/Axis/JoystickAxisExpo.h>

/**
 * Imagine you have a literal tank, and you have 2 pedals. One for the left wheels
 * and one for the right wheels. This is literally the same.
 * Left Joystick = Left wheels
 * Right Joystick = Right wheels
 */


struct EctoTankInputHandlerConfig {
	double maxLinearVelocity = 5.0;
	double maxAngularVelocity = M_PI;
};

class EctoTankInputHandler : public System {
public:
	EctoTankInputHandler(const std::shared_ptr<EctoTank> &tank, const EctoTankInputHandlerConfig &config);
	
	void robotInit() override;
	
	void robotUpdate() override;

private:
	//CONTROL INPUTS
	JoystickAxisExpo xLeftJoy{0.1, 0.1};
	JoystickAxisExpo xRightJoy{0.1, 0.1};
	
	//CONTROL INPUT NAMES
	static constexpr auto rightXJoyName = "rightX";
	static constexpr auto leftXJoyName = "leftX";
	
	//INSTANCES
	EctoTankInputHandlerConfig config;
	InputManager &input = InputManager::getInstance();
	std::shared_ptr<EctoTank> tank;
	
};


#endif //BOTBUSTERSREBIRTH_ECTOTANKINPUTHANDLER_H
