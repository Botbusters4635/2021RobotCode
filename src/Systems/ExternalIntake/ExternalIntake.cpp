//
// Created by abiel on 1/6/20.
//

#include "ExternalIntake.h"

ExternalIntake::ExternalIntake(const ExternalIntakeConfig &config, std::string externalIntakeName) : System(
		externalIntakeName) {
	this->config = config;
	this->externalIntakeName = externalIntakeName;
	piston = pcm.getPiston(config.pistonName);
	
	intakeMotor = config.intakeMotor;
}

void ExternalIntake::liftIntake(bool state) {
	piston->Set(state ? frc::DoubleSolenoid::kForward : frc::DoubleSolenoid::kReverse);
}

void ExternalIntake::setMotor(double value) {
	intakeMotor->set(value);
}

void ExternalIntake::robotInit() {
	intakeMotor->setMotorCurrentLimit(config.motorCurrentLimit);
	intakeMotor->enableCurrentLimit(true);
	intakeMotor->setOpenLoopRampRate(config.motorOpenLoopRampRate);
	intakeMotor->setControlMode(MotorControlMode::Percent);
	
	table = ntInstance.GetTable(externalIntakeName);
}

bool ExternalIntake::isIntakeDown() const {
	return piston->Get();
}

void ExternalIntake::updateNetworkTables() {
	table->GetEntry("CurrentSetpoint").SetDouble(intakeMotor->getLastSetpoint().second);
	table->GetEntry("IsIntakeDown").SetBoolean(isIntakeDown());
}

void ExternalIntake::robotUpdate() {
	updateNetworkTables();
}