//
// Created by abiel on 11/3/21.
//

#include "SwerveCharacterizationRobot.h"

SwerveCharacterizationRobot::SwerveCharacterizationRobot() : EctoRobot("SwerveCharacterizationRobot") {
	;
}

void SwerveCharacterizationRobot::robotInit() {
	EctoSwerveConfig swerveConfig;
	swerveConfig.length = 0.5334;
	swerveConfig.width = 0.5334;
	swerveConfig.wheelCircumference = 0.0508 * 2 * M_PI;
	swerveConfig.gearRatio = 8.95;
	swerve = std::make_shared<EctoSwerve>(swerveConfig);
	
	auto &input = InputManager::getInstance();
	
	systemManager->addSubsystem(swerve);
	input.registerAxis(leftSide, "leftY");
	input.registerAxis(rightSide, "rightY");
}

void SwerveCharacterizationRobot::teleopUpdate() {
	swerve->setModules(tankSwerve(leftSide.get() * 12.0, rightSide.get() * 12.0), MotorControlMode::Voltage);
}

void SwerveCharacterizationRobot::autoInit() {
	logger.InitLogging();
}

void SwerveCharacterizationRobot::autoUpdate() {
	auto state = swerve->getMotorStates();
	
	auto leftPos = (state.topLeft.wheelPosition + state.backLeft.wheelPosition) / 2.0;
	auto rightPos = (state.topRight.wheelPosition + state.backRight.wheelPosition) / 2.0;
	
	auto leftVel = (state.topLeft.wheelVelocity + state.backLeft.wheelVelocity) / 2.0;
	auto rightVel = (state.topRight.wheelVelocity + state.backRight.wheelVelocity) / 2.0;
	
	auto yaw = swerve->getYaw();
	auto angularRate = swerve->getYawRate();
	
	logger.Log(
			leftPos, rightPos, leftVel, rightVel, yaw, angularRate
	);
	
	double left = logger.GetLeftMotorVoltage().value();
	double right = logger.GetRightMotorVoltage().value();
	
	swerve->setModules(tankSwerve(left, right), MotorControlMode::Voltage);
}

void SwerveCharacterizationRobot::disabledInit() {
	swerve->setModules(tankSwerve(0, 0));
	logger.SendData();
}