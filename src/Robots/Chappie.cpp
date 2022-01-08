//
// Created by Neil Rodriguez Murillo on 9/23/21.
//


#include <Core/EctoInput/DefaultMappings.h>
#include "Chappie.h"

Chappie::Chappie() : EctoRobot("Chappie") {
	;
}

void Chappie::robotInit() {
	EctoTankConfig tankConfig;
	IntakeConfig intakeConfig;
	
	//intake setup
	intakeConfig.piston = pcm.getPiston("intake");
	intakeConfig.motor = manager.getMotor("intakeMotor");
	
	//tank setup
	tankConfig.leftMotors.push_back(manager.getMotor("leftMasterMotor"));
	tankConfig.leftMotors.push_back(manager.getMotor("leftSlaveMotor0"));
	tankConfig.leftMotors.push_back(manager.getMotor("leftSlaveMotor1"));
	
	tankConfig.rightMotors.push_back(manager.getMotor("rightMasterMotor"));
	tankConfig.rightMotors.push_back(manager.getMotor("rightSlaveMotor0"));
	tankConfig.rightMotors.push_back(manager.getMotor("rightSlaveMotor1"));
	
	systemManager->addSubsystem(tank);
	inputHandlers->addSubsystem(tankInputHandler);
	
	systemManager->addSubsystem(intake);
	inputHandlers->addSubsystem(intakeInputHandler);
	
}


void Chappie::autoInit() {
}

void Chappie::autoUpdate() {
}

void Chappie::teleopInit() {
}

void Chappie::teleopUpdate() {

}

std::list<PistonInfo> Chappie::getPistonConfig() {
	//TODO IDS
	return {
			{"intake", 0, 0}
	};
}


std::list<MotorInfo> Chappie::getMotorConfig() {
//TOOO IDS
	return {
			
			//Tank config
			{EctoMotorType::SparkMax, "leftMasterMotor",  0},
			{EctoMotorType::SparkMax, "leftSlaveMotor0",  0},
			{EctoMotorType::SparkMax, "leftSlaveMotor1",  0},
			{EctoMotorType::SparkMax, "rightMasterMotor", 0},
			{EctoMotorType::SparkMax, "rightSlaveMotor0", 0},
			{EctoMotorType::SparkMax, "rightSlaveMotor1", 0},
			
			//Intake Config
			{EctoMotorType::SparkMax, "intakeMotor",      0},
		
	};
}

