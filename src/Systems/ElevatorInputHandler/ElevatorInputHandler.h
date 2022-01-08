//
// Created by Alberto Jahueyu on 2/25/2020.
//

#ifndef BOTBUSTERS_REBIRTH_ELEVATORINPUTHANDLER_H
#define BOTBUSTERS_REBIRTH_ELEVATORINPUTHANDLER_H

#include "Systems/Elevator/Elevator.h"
#include "Core/EctoInput/InputManager.h"
#include <Core/EctoInput/Axis/JoystickAxisExpo.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>

struct ElevatorInputHandlerConfig {
	std::shared_ptr<Elevator> elevator;
	double maxForwardOutput = 1.0;
	double maxReverseOutput = -1.0;
};

class ElevatorInputHandler : public System {
public:
	ElevatorInputHandler(ElevatorInputHandlerConfig config);
	
	void robotInit() override;
	
	void robotUpdate() override;

private:
	nt::NetworkTableInstance ntInstance = nt::NetworkTableInstance::GetDefault();
	std::shared_ptr<nt::NetworkTable> table;
	
	InputManager &input = InputManager::getInstance();
	ElevatorInputHandlerConfig config;
	JoystickAxisExpo elevatorControl{0.25, 0.05};
	EctoButton elevatorToggle;
	bool lastElevatorToggle = false;
};


#endif //BOTBUSTERS_REBIRTH_ELEVATORINPUTHANDLER_H
