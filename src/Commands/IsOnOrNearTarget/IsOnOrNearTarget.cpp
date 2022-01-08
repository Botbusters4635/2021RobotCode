//
// Created by cc on 23/11/21.
//

#include "IsOnOrNearTarget.h"
#include <frc/geometry/Pose2d.h>


IsOnOrNearTarget::IsOnOrNearTarget(const std::shared_ptr<EctoSwerve> &_swerve, double _tX, double _tY, double _tol) {
	tX = _tX;
	tY = _tY;
	tol = _tol;
	swerve = _swerve;
}

void IsOnOrNearTarget::Initialize() { ; }

void IsOnOrNearTarget::Execute() { ; }

void IsOnOrNearTarget::End(bool interrupted) { ; }


bool IsOnOrNearTarget::IsFinished() {
	auto swervePose = swerve->getPose();
	double distance = std::hypot(tX - swervePose.X().value(), tY - swervePose.Y().value());
	return distance < tol;
}

