//
// Created by abiel on 2/17/20.
//

#include "LowerIntake.h"

LowerIntake::LowerIntake(const std::shared_ptr<Intake> &intake, double targetPct) {
	this->intake = intake;
	this->targetPct = targetPct;
	
}

void LowerIntake::Initialize() {
	intake->set(targetPct, true);
}

void LowerIntake::Execute() {
	;
}

void LowerIntake::End(bool interrupted) {
	;
}

bool LowerIntake::IsFinished() {
	return true;
}