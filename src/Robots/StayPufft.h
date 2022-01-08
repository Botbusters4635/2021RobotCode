//
// Created by 4ndre on 22/09/2021.
//

#ifndef BOTBUSTERS_REBIRTH_STAYPUFFT_H
#define BOTBUSTERS_REBIRTH_STAYPUFFT_H

#include <networktables/NetworkTableInstance.h>
#include <Core/EctoInput/Axis/JoystickAxisExpo.h>
#include <Systems/EctoSwerve/EctoSwerve.h>
#include <Systems/EctoSwerve/EctoSwerveInputHandler.h>
#include "Core/EctoRobot.h"
#include <frc/controller/HolonomicDriveController.h>
#include <frc/trajectory/TrajectoryConfig.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/trajectory/constraint/SwerveDriveKinematicsConstraint.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc2/command/FunctionalCommand.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc2/command/SwerveControllerCommand.h>
#include "Systems/PIDShooter/PIDShooter.h"
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc2/command/Subsystem.h>
#include <Math/SystemInterpolatingTable/InterpolatingTable.h>
#include "Systems/PIDShooter/PIDShooter.h"
#include "Systems/Intake/Intake.h"
#include "Systems/Intake/IntakeInputHandler.h"
#include "Control/PathFollowers/PathFollowerRotationProfile.h"
#include "Commands/UseIntake/UseIntake.h"
#include "Commands/LowerIntake/LowerIntake.h"
#include "Core/VisionManager/VisionManager.h"


class StayPufft : public EctoRobot {
public:
	StayPufft();
	
	void disabledInit() override;
	
	void disabledUpdate() override;
	
	void robotInit() override;
	
	void robotUpdate() override;
	
	void autoInit() override;
	
	void autoUpdate() override;
	
	void teleopInit() override;
	
	void teleopUpdate() override;

protected:
	std::list<MotorInfo> getMotorConfig() override {
		return {{EctoMotorType::SparkMax, "front_left_wheel",  1},
		        {EctoMotorType::SparkMax, "front_right_wheel", 3},
		        {EctoMotorType::SparkMax, "back_left_wheel",   7},
		        {EctoMotorType::SparkMax, "back_right_wheel",  6},
		
		        {EctoMotorType::SparkMax, "front_left_steer",  2},
		        {EctoMotorType::SparkMax, "front_right_steer", 4},
		        {EctoMotorType::SparkMax, "back_left_steer",   8},
		        {EctoMotorType::SparkMax, "back_right_steer",  5},
		
		        {EctoMotorType::SparkMax, "intake",            9},
		};
	}
	
	std::list<PistonInfo> getPistonConfig() override {
		return {{"intake", 0, 1}};
	}

private:
	std::shared_ptr<EctoSwerve> swerve;
	std::shared_ptr<EctoSwerveInputHandler> swerveInputHandler;
	InputManager &input = InputManager::getInstance();
	MotorManager &handler = MotorManager::getInstance();
	
	PIDShooterConfig configShooter;
	std::shared_ptr<PIDShooter> shooterPID;
	
	std::shared_ptr<Intake> intake;
	std::shared_ptr<IntakeInputHandler> intakeInputHandler;

	nt::NetworkTableInstance ntInstance = nt::NetworkTableInstance::GetDefault();
	std::shared_ptr<nt::NetworkTable> table;
	
	InterpolatingTable interTable;
	
	units::radians_per_second_t maxVelocity_rot{1};
	units::radians_per_second_squared_t maxAcceleration_rot{1};
	
	frc::TrapezoidProfile<units::radian>::Constraints constraints = frc::TrapezoidProfile<units::radian>::Constraints(
			maxVelocity_rot, maxAcceleration_rot);
	wpi::ArrayRef<frc2::Subsystem *> requirements;

	frc2::Command *sequentialCommand;

	std::shared_ptr<VisionManager> visionManager;
	std::unique_ptr<PathFollowerRotationProfile> rotationProfile;
	
	frc::Trajectory trajectory;
	
};


#endif //BOTBUSTERS_REBIRTH_STAYPUFFT_H
