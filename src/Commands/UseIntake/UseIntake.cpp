//
// Created by Neil Rodriguez Murillo on 10/11/21.
//

#include "UseIntake.h"

UseIntake::UseIntake(const std::shared_ptr<Intake> &intake, double TargetPct, double state) {
	this->intake = intake;
	this->targetPct = TargetPct;
	this->state = state;
}

void UseIntake::Initialize() {
	intake->set(targetPct, state);
}

void UseIntake::Execute() {
	;
}

void UseIntake::End(bool interrupted) {
	;
}

bool UseIntake::IsFinished() {
	return true;
}