//
// Created by abiel on 11/3/21.
//

#ifndef BOTBUSTERS_REBIRTH_HOLONOMICPATHFOLLOWER_H
#define BOTBUSTERS_REBIRTH_HOLONOMICPATHFOLLOWER_H

#include "Systems/EctoSwerve/EctoSwerve.h"
#include <frc/trajectory/Trajectory.h>
#include "Control/Feedforward/HolonomicFeedforward.h"
#include <frc2/command/CommandHelper.h>
#include <frc2/command/CommandBase.h>
#include <frc/controller/ProfiledPIDController.h>

#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableInstance.h"
#include <functional>

struct HolonomicPathFollowerConfig {
	HolonomicFeedforward ff;
	std::function<frc::Rotation2d(double x, double y)> rotationFunction = [](double x, double y) {
		return frc::Rotation2d();
	};
	
	bool runUntilPathFinished = true;
	bool stopWhenFinished = true;
	
	PIDConfig posConfig;
	PIDConfig thetaConfig;
	
	double maxAngularVelocity{7}, maxAngularAcceleration{8};
	
	double endTolerance = 0.1;
	
	void setEndRotation(const frc::Rotation2d &rotIn) {
		rotationFunction = [rotIn](double, double) { return rotIn; };
	}
};

class HolonomicPathFollower : public frc2::CommandHelper<frc2::CommandBase, HolonomicPathFollower> {
public:
	HolonomicPathFollower(const std::shared_ptr<EctoSwerve> &swerve, const frc::Trajectory &traj,
	                      const HolonomicPathFollowerConfig &config);
	
	void Initialize() override;
	
	void Execute() override;
	
	void End(bool interrupted) override;
	
	bool IsFinished() override;

private:
	std::shared_ptr<EctoSwerve> swerve;
	frc::Trajectory traj;
	
	std::unique_ptr<frc2::PIDController> xController, yController;
	std::unique_ptr<frc::ProfiledPIDController<units::radian>> thetaController;
	double startTime{0};
	
	void
	updateTelemetry(const frc::Trajectory::State &state, const frc::Vector2d &ffVec, const frc::ChassisSpeeds &out);
	
	double getTime() {
		return frc::Timer::GetFPGATimestamp() - startTime;
	}
	
	frc::Vector2d fromAngle(double heading, double mag) {
		frc::Vector2d out(std::cos(heading), std::sin(heading)); //TODO check if sin/cos
		out.x *= mag;
		out.y *= mag;
		return out;
	}
	
	HolonomicPathFollowerConfig config;
	HolonomicFeedforward ff;
	
	std::shared_ptr<nt::NetworkTable> table;
	nt::NetworkTableEntry ffXOutEntry, ffYOutEntry;
	nt::NetworkTableEntry xOutEntry, yOutEntry, thetaOutEntry;
	nt::NetworkTableEntry velEntry, accelEntry;
	nt::NetworkTableEntry pathStateXEntry, pathStateYEntry;
};


#endif //BOTBUSTERS_REBIRTH_HOLONOMICPATHFOLLOWER_H
