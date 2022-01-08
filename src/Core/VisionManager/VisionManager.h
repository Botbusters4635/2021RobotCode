//
// Created by abiel on 2/12/20.
//

#ifndef BOTBUSTERSREBIRTH_VISIONMANAGER_H
#define BOTBUSTERSREBIRTH_VISIONMANAGER_H

#include "Core/EctoModule/System.h"
#include <frc/Timer.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include "Systems/EctoSwerve/EctoSwerve.h"
#include <photonlib/PhotonCamera.h>
#include <photonlib/PhotonUtils.h>
#include <photonlib/PhotonTrackedTarget.h>
#include <optional>
#include <functional>

class VisionManager : public System {
public:
	VisionManager(const std::shared_ptr<EctoSwerve> &swerve);
	
	void robotInit() override;
	
	void robotUpdate() override;
	
	photonlib::PhotonPipelineResult getLatestResult();

private:
	std::shared_ptr<EctoSwerve> swerve;
	
	photonlib::PhotonCamera camera{"gloworm"};
	photonlib::PhotonPipelineResult prevResult;
	
	nt::NetworkTableInstance ntInstance = nt::NetworkTableInstance::GetDefault();
	std::shared_ptr<nt::NetworkTable> table, visionTable, photonVisionTable;
	
	photonlib::PhotonTrackedTarget target;
	
	frc::Pose2d targetPose{0.68_m, 7.44_m, 180_deg};
	frc::Transform2d cameraPoseInRobot{frc::Translation2d(-0.26_m, 0_m), 0_deg};
	
	double targetTimestamp;
	
	bool isValidTarget() const {
		return frc::Timer::GetFPGATimestamp() - targetTimestamp < 100; //Invalid target if has not updated within 100ms
	}
	
	void updateVision(const photonlib::PhotonPipelineResult &result);
	
	void updateVisionTelemetry(const photonlib::PhotonPipelineResult &result);
	
	void updateTelemetry();
	
	frc::SwerveDrivePoseEstimator<4> *poseEstimator;
	
	photonlib::PhotonPipelineResult result;
};

#endif //BOTBUSTERSREBIRTH_VISIONMANAGER_H
