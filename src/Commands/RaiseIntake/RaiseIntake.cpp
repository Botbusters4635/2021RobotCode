
//
// Created by cc on 06/12/21.
//

#include "RaiseIntake.h"

RaiseIntake::RaiseIntake(const std::shared_ptr<Intake> &intake, double targetPct) {
	this->intake = intake;
	this->targetPct = targetPct;
	
}

void RaiseIntake::Initialize() {
	intake->set(targetPct, false);
}

void RaiseIntake::Execute() {
	;
}

void RaiseIntake::End(bool interrupted) {
	;
}

bool RaiseIntake::IsFinished() {
	return true;
}