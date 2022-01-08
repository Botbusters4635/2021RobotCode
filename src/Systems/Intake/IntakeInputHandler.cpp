//
// Created by Neil Rodriguez Murillo on 10/8/21.
//

#include "IntakeInputHandler.h"
#include <networktables/NetworkTableInstance.h>
#include <frc/smartdashboard/SmartDashboard.h>


void IntakeInputHandler::robotInit() {
	input.registerButton(leftShoulder, intakePistonButton, 1);
	input.registerButton(rightShoulder, intakePistonAndSpitButton, 1);
	input.registerAxis(xRightJoy, intakeJoy, 1);
}

void IntakeInputHandler::robotUpdate() {
	auto vol = xRightJoy.get();
	auto state = leftShoulder.get();
	auto spit = rightShoulder.get();
	if (spit == true) {
		intake->set(vol = -0.6, state = false);
	}
//    frc::SmartDashboard::PutNumber("Debug/Intake/vol", vol);
//    frc::SmartDashboard::PutBoolean("Debug/Intake/state", state);
	if (state == true && vol == 0) {
		intake->set(vol = 0.8, state);
	}
//    if (state != true && vol == 0){
//        intake->set(vol = 0.14, state);
//    }
	intake->set(vol, state);
}
