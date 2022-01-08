//
// Created by cc on 04/01/22.
//

#include "VisionAlign.h"

VisionAlign::VisionAlign(const std::shared_ptr<EctoSwerve> &swerve,
                         const std::shared_ptr<VisionManager> &visionManager) {
	this->visionManager = visionManager;
	this->swerve = swerve;
	visionTable = ntInstance.GetTable("photonvision/gloworm");
	visionAngle = visionTable->GetEntry("targetYaw");
	hasTarget = visionTable->GetEntry("hasTarget");
	
	
}

void VisionAlign::Initialize() {
	;
}

void VisionAlign::Execute() {
	frc::SmartDashboard::PutBoolean("hasTarget", hasTarget.GetBoolean(false));
	
	if (hasTarget.GetBoolean(false)) {
//        state = visionManager->getLatestResult().GetBestTarget().GetYaw();
		state = visionAngle.GetDouble(0);
		visionOut = visionAnglePID.Calculate(state, setPoint);
		chassisSpeeds.omega = units::radians_per_second_t(visionOut);
		swerve->setPercent(chassisSpeeds);
		frc::SmartDashboard::PutNumber("cameraYaw", state);
	}
	
}

void VisionAlign::End(bool interrupted) {
	chassisSpeeds.omega = units::radians_per_second_t(0);
	swerve->setPercent(chassisSpeeds);
}

bool VisionAlign::IsFinished() {
	if (std::abs(setPoint - state) < tol) {
		return true;
	} else {
		return false;
	}
}