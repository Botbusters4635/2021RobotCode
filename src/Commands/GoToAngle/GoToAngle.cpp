//
// Created by cc on 06/01/22.
//

#include "GoToAngle.h"

GoToAngle::GoToAngle(const std::shared_ptr<EctoSwerve> &swerve, double angle, double tol) {
	this->swerve = swerve;
	this->angle = angle * (M_PI / 180.0);
	this->tol = tol;
	
}

void GoToAngle::Initialize() {
	anglePID.EnableContinuousInput(-M_PI, M_PI);
}

void GoToAngle::Execute() {
	state = swerve->getYaw();
	pidOut = anglePID.Calculate(state, angle);
	chassisSpeeds.omega = units::radians_per_second_t(pidOut);
	swerve->setPercent(chassisSpeeds);
}

void GoToAngle::End(bool interrupted) {
	chassisSpeeds.omega = units::radians_per_second_t(0);
	swerve->setPercent(chassisSpeeds);
}

bool GoToAngle::IsFinished() {
	if (std::abs(angle - state) < tol) {
		return true;
	} else {
		return false;
	}
}