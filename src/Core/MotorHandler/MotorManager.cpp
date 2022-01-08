//
// Created by alberto on 31/07/19.
//

#include "MotorManager.h"
#include "EctoMotor/EctoTalon.h"
#include "EctoMotor/EctoSpark.h"
#include "EctoMotor/EctoPWM.h"
#include "EctoMotor/EctoGazeboMotor.h"
#include <iostream>


MotorManager::MotorManager() : Manager("MotorHandler") {
	log->info("Initializing EctoMotors...");
}

void MotorManager::init() {
	initializeMotors();
	hasInit = true;
}

void MotorManager::setMotorInfo(const std::list<MotorInfo> &motorInfo) {
	this->motorInfo = motorInfo;
}

void MotorManager::initializeMotors() {
	for (const auto &motorData: motorInfo) {
		std::shared_ptr<EctoMotor> motor;
		std::string fwVer;
#ifndef SIMULATION
		switch (motorData.type) {
			case EctoMotorType::SparkMax:
				motor = std::make_shared<EctoSpark>(motorData.id, motorData.name);
				fwVer = motor->getFirmwareVersion();
				if (fwVer != sparkMaxVersion) {
					log->error("SparkMax: {} with name: {} has different FW version: {}", motorData.id, motorData.name,
					           fwVer);
				}
				break;
				//TODO define other motors
			default:
				throw std::runtime_error("Invalid motor type defined!");
		}
#else
		motor = std::make_shared<EctoGazeboMotor>("SimRobot", motorData.name);
#endif
		motorControllers.emplace_back(motor);
	}
}


void MotorManager::update() {
	;
}

std::shared_ptr<EctoMotor> &MotorManager::getMotor(const std::string &name) {
	if (!hasInit) throw std::runtime_error("MotorManager has not been initialized!");
	
	for (auto &motor: motorControllers) {
		//log->info("Motor Name: {}", motor->getName());
		if (motor->getName() == name) {
			return motor;
		}
	}
	throw std::invalid_argument("Could not get motor with the name '" + name + "', it does not exist");
}
