//
// Created by Alberto Jahueyu on 2/25/2020.
//

#include "ElevatorInputHandler.h"

ElevatorInputHandler::ElevatorInputHandler(ElevatorInputHandlerConfig config) : System("ElevatorInputHandler") {
	this->config = config;
	input.registerAxis(elevatorControl, "rightX", 2);
	input.registerButton(elevatorToggle, "select", 2);
	table = ntInstance.GetTable("ElevatorInputHandler");
}

void ElevatorInputHandler::robotInit() {
}

void ElevatorInputHandler::robotUpdate() {
	bool toggleState = table->GetEntry("ElevatorToggle").GetBoolean(false);
	
	if (elevatorToggle.get() != lastElevatorToggle && elevatorToggle.get()) {
		table->GetEntry("ElevatorToggle").SetBoolean(!toggleState);
	}
	
	if (toggleState) {
		double output = -elevatorControl.get();
		
		output = output < config.maxReverseOutput ? config.maxReverseOutput : output;
		output = output > config.maxForwardOutput ? config.maxForwardOutput : output;
		config.elevator->climb(output);
	} else {
		config.elevator->climb(0.0);
	}
	
	lastElevatorToggle = elevatorToggle.get();
}
