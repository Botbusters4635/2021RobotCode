//
// Created by abiel on 2/12/20.
//

#include "VisionManager.h"
#include "frc/StateSpaceUtil.h"

VisionManager::VisionManager(const std::shared_ptr<EctoSwerve> &swerve) : System("VisionManager") {
	this->swerve = swerve;
	
	//TODO @gus make initial gyro and pose configurable
	//TODO @gus make std devs configurable
	//https://docs.wpilib.org/en/stable/docs/software/advanced-controls/state-space/state-space-pose_state-estimators.html
	poseEstimator = new frc::SwerveDrivePoseEstimator<4>(
			frc::Rotation2d(), //Initial gyro angle
			frc::Pose2d(), //Initial robot pose
			*swerve->getKinematics(),
			{0.0, 0.0, 0.0}, //State std devs {x,y,theta}
			{0.0}, //Local (gyro, encoder) std devs {theta}
			{0.0, 0.0, 0.0} //Global (vision) std devs {theta}
	);
}

void VisionManager::robotInit() {
	table = ntInstance.GetTable("VisionManager");
	visionTable = ntInstance.GetTable("VisionManager/Vision");
	photonVisionTable = ntInstance.GetTable("photonvision/gloworm");
}

void VisionManager::robotUpdate() {
	result = camera.GetLatestResult();
	
	if (result != prevResult) {
		//Update vision result
		updateVision(result);
		updateVisionTelemetry(result);
		prevResult = result;
	}
	
	auto gyroAngle = swerve->getPose().Rotation();
	auto moduleStates = swerve->getMotorStates().toWPI();
	
	//TODO {season} @gus make it so that std devs can be changed for teleop and auto
	poseEstimator->Update(gyroAngle, moduleStates);
	
	updateTelemetry();
}

void VisionManager::updateVision(const photonlib::PhotonPipelineResult &result) {
	if (!result.HasTargets()) return; //No targets found
	target = result.GetTargets()[0];
	visionTable->GetEntry("result X").SetDouble(result.GetBestTarget().GetCameraRelativePose().X().value());
	visionTable->GetEntry("result Y").SetDouble(result.GetBestTarget().GetCameraRelativePose().Y().value());
	visionTable->GetEntry("result Angle").SetDouble(
			result.GetBestTarget().GetCameraRelativePose().Rotation().Degrees().value());
	
	targetTimestamp = frc::Timer::GetFPGATimestamp() - result.GetLatency().value();
	frc::Transform2d camToTargetTrans = target.GetCameraRelativePose();
	std::vector<double> targetPoseArray = photonVisionTable->GetNumberArray("targetPose", {});
	if (targetPoseArray.size() != 3) {
		return;
	}
	
	
	camToTargetTrans = frc::Transform2d(
			frc::Translation2d(units::meter_t(targetPoseArray[0]), units::meter_t(-targetPoseArray[1])),
			frc::Rotation2d(units::degree_t(-targetPoseArray[2])));
	visionTable->GetEntry("camToTargetTrans X").SetDouble(camToTargetTrans.X().value());
	visionTable->GetEntry("camToTargetTrans Y").SetDouble(camToTargetTrans.Y().value());
	visionTable->GetEntry("camToTargetTrans Heading").SetDouble(camToTargetTrans.Rotation().Degrees().value());
	
	frc::Pose2d camPose = targetPose.TransformBy(camToTargetTrans.Inverse());
	visionTable->GetEntry("camPose X").SetDouble(camPose.X().value());
	visionTable->GetEntry("camPose Y").SetDouble(camPose.Y().value());
	visionTable->GetEntry("camPose Heading").SetDouble(camPose.Rotation().Degrees().value());

//    frc::Pose2d robotPose = camPose.TransformBy(cameraPoseInRobot);
//    frc::Translation2d translation = photonlib::PhotonUtils::EstimateCameraToTargetTranslation();
	
	frc::Pose2d robotPose = photonlib::PhotonUtils::EstimateFieldToRobot(0.36_m, 1.685_m, -32.8_deg,
	                                                                     units::radian_t(target.GetPitch()),
	                                                                     units::radian_t(-target.GetYaw()),
	                                                                     swerve->getPose().Rotation(), targetPose,
	                                                                     cameraPoseInRobot);
	visionTable->GetEntry("robotPose X").SetDouble(robotPose.X().value());
	visionTable->GetEntry("robotPose Y").SetDouble(robotPose.Y().value());
	visionTable->GetEntry("robotPose Heading").SetDouble(robotPose.Rotation().Degrees().value());
	
	poseEstimator->AddVisionMeasurement(robotPose, units::second_t(targetTimestamp));
	
	
	//TODO @gus {season} make std devs change in auto or teleop
	//poseEstimator->AddVisionMeasurement(visionPose, units::second_t(targetTimestamp));
}

void VisionManager::updateVisionTelemetry(const photonlib::PhotonPipelineResult &result) {
	visionTable->GetEntry("HasTargets").SetBoolean(result.HasTargets());
	if (result.HasTargets()) {
		visionTable->GetEntry("TargetCount").SetDouble(result.GetTargets().size());
	}
	
	visionTable->GetEntry("Latency").SetDouble(result.GetLatency().value() / 1000.0);
	
	if (isValidTarget()) {
		visionTable->GetEntry("Yaw").SetDouble(target.GetYaw());
	}
}

void VisionManager::updateTelemetry() {
	visionTable->GetEntry("EstimatedRobotPose X").SetDouble(poseEstimator->GetEstimatedPosition().X().value());
	visionTable->GetEntry("EstimatedRobotPose Y").SetDouble(poseEstimator->GetEstimatedPosition().Y().value());
	visionTable->GetEntry("EstimatedRobotPose Theta").SetDouble(
			poseEstimator->GetEstimatedPosition().Rotation().Degrees().value());
	//TODO @gus send pose estimator telemetry to nt {table}
}


photonlib::PhotonPipelineResult VisionManager::getLatestResult() {
	return result;
}