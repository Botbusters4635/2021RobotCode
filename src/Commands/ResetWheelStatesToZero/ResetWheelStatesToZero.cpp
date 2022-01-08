//
// Created by cc on 06/01/22.
//

#include "ResetWheelStatesToZero.h"

ResetWheelStateToZero::ResetWheelStateToZero(const std::shared_ptr<EctoSwerve> &swerve) {
	this->swerve = swerve;
}

void ResetWheelStateToZero::Initialize() {
	;
}

void ResetWheelStateToZero::Execute() {
	std::array<frc::SwerveModuleState, 4> states;
	swerve->setModules(states);
	swerve->zeroOdometry();
}

void ResetWheelStateToZero::End(bool interrupted) {
	;
}

bool ResetWheelStateToZero::IsFinished() {
	return true;
}