//
// Created by 4ndre on 22/09/2021.
//


#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/InstantCommand.h>
#include "StayPufft.h"
#include <frc2/command/FunctionalCommand.h>
#include "Control/PathFollowers/HolonomicPathFollower.h"
#include "Commands/IsOnOrNearTarget/IsOnOrNearTarget.h"
#include "Commands/LowerIntake/LowerIntake.h"
#include "Commands/RaiseIntake/RaiseIntake.h"
#include "Commands/VisionAlign/VisionAlign.h"
#include "Commands/GoToAngle/GoToAngle.h"
#include "Commands/ResetWheelStatesToZero/ResetWheelStatesToZero.h"


StayPufft::StayPufft() : EctoRobot("StayPufft") {
	
	
	table = ntInstance.GetTable("StayPufft");
}

void StayPufft::disabledInit() {

}

void StayPufft::disabledUpdate() {

}

void StayPufft::robotInit() {
	
	EctoSwerveConfig swerveConfig;
	swerveConfig.length = 0.5334;
	swerveConfig.width = 0.5334;
	swerveConfig.wheelCircumference = 0.0508 * 2 * M_PI;
	swerveConfig.gearRatio = 8.95;
	swerve = std::make_shared<EctoSwerve>(swerveConfig);
	swerveInputHandler = std::make_shared<EctoSwerveInputHandler>(*(swerve));
	visionManager = std::make_shared<VisionManager>(swerve);
	
	systemManager->addSubsystem(swerve);
	inputHandlers->addSubsystem(swerveInputHandler);
	systemManager->addSubsystem(visionManager);
	
	IntakeConfig intakeConfig;
	intakeConfig.motor = motorManager.getMotor("intake");
	intakeConfig.piston = pcmManager.getPiston("intake");
	intakeConfig.invertMotor = true;
	intake = std::make_shared<Intake>(intakeConfig);
	intakeInputHandler = std::make_shared<IntakeInputHandler>(intake);
	
	systemManager->addSubsystem(intake);
	inputHandlers->addSubsystem(intakeInputHandler);
}

void StayPufft::robotUpdate() {
}

void StayPufft::autoInit() {
}

void StayPufft::autoUpdate() {

}

void StayPufft::teleopInit() {

}

void StayPufft::teleopUpdate() {

}
