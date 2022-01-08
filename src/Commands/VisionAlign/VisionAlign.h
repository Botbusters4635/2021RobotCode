//
// Created by cc on 04/01/22.
//

#ifndef BOTBUSTERS_REBIRTH_VISIONALIGN_H
#define BOTBUSTERS_REBIRTH_VISIONALIGN_H

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "Systems/EctoSwerve/EctoSwerve.h"
#include <photonlib/PhotonCamera.h>
#include "Core/VisionManager/VisionManager.h"
#include <limits>

class VisionAlign : public frc2::CommandHelper<frc2::CommandBase, VisionAlign> {
public:
	VisionAlign(const std::shared_ptr<EctoSwerve> &swerve, const std::shared_ptr<VisionManager> &visionManager);
	
	void Initialize() override;
	
	void Execute() override;
	
	void End(bool interrupted) override;
	
	bool IsFinished() override;


private:
	std::shared_ptr<EctoSwerve> swerve;
	std::shared_ptr<VisionManager> visionManager;
	frc::ChassisSpeeds chassisSpeeds;
	
	double setPoint = 0;
	double state = std::numeric_limits<double>::max();
	double visionOut;
	bool isFinished;
	double tol = 5;
	
	nt::NetworkTableInstance ntInstance = nt::NetworkTableInstance::GetDefault();
	std::shared_ptr<nt::NetworkTable> visionTable;
	nt::NetworkTableEntry visionAngle, hasTarget;
	
	frc2::PIDController visionAnglePID{0.022, 0, 0.00095};
	
};


#endif //BOTBUSTERS_REBIRTH_VISIONALIGN_H
