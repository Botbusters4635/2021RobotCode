//
// Created by abiel on 1/6/20.
//

#include "BallFeeder.h"

BallFeeder::BallFeeder(const BallFeederConfig &config) : System("BallFeeder") {
	this->config = config;
	motor = config.motor;
	
	ballSwitch = std::make_shared<frc::DigitalInput>(2);
	ballCounter = std::make_unique<frc::Counter>(ballSwitch);
	
	glitchFilter.SetPeriodNanoSeconds(40000000);
	
	glitchFilter.Add(ballSwitch.get());
}

void BallFeeder::robotInit() {
	motor->setMotorCurrentLimit(25);
	motor->enableCurrentLimit(true);
	motor->setControlMode(MotorControlMode::Percent);
	motor->setOpenLoopRampRate(config.feederOpenLoopRampRate);
	motor->enableBrakingOnIdle(false);
	
	table = ntInstance.GetTable("BallFeeder");
}

void BallFeeder::feedBalls(double feedSpeed) {
	if (stopAtSwitch) {
		if (ballSwitch->Get()) {
			motor->set(feedSpeed);
		} else {
			motor->set(0);
		}
	} else {
		motor->set(feedSpeed);
	}
}

void BallFeeder::stopFeeding() {
	motor->set(0);
}

void BallFeeder::robotUpdate() {
	updateNetworkTables();
}

void BallFeeder::updateNetworkTables() {
	table->GetEntry("CurrentSetpoint").SetDouble(motor->getLastSetpoint().second);
	table->GetEntry("LimitSwitch").SetBoolean(ballSwitch->Get());
	table->GetEntry("StopAtSwitch").SetBoolean(stopAtSwitch);
	table->GetEntry("BallCount").SetDouble(getBallCount());
}

void BallFeeder::stopBallAtSwitch(bool state) {
	stopAtSwitch = state;
}

int BallFeeder::getBallCount() {
	return ballCounter->Get();
}

void BallFeeder::resetBallCount() {
	ballCounter->Reset();
}

bool BallFeeder::isBallPreloaded() const {
	return !ballSwitch->Get();
}