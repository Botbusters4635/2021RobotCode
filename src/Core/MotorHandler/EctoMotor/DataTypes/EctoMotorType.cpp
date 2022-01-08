//
// Created by hiram on 9/08/19.
//

#include <stdexcept>
#include "EctoMotorType.h"

std::string getStringFromMotorType(EctoMotorType type) {
	switch (type) {
		case EctoMotorType::PWM:
			return "PWM";
		case EctoMotorType::TalonSRX:
			return "TalonSRX";
		case EctoMotorType::SparkMax:
			return "SparkMax";
		case EctoMotorType::SparkMaxBrushed:
			return "SparkMaxBrushed";
		case EctoMotorType::Empty:
			return "Empty";
		default:
			return "InvalidType";
	}
}

EctoMotorType getMotorTypeFromString(const std::string &type) {
	if (type == "pwm") return EctoMotorType::PWM;
	else if (type == "talonsrx") return EctoMotorType::TalonSRX;
	else if (type == "sparkmax") return EctoMotorType::SparkMax;
	else if (type == "sparkmaxbrushed") return EctoMotorType::SparkMaxBrushed;
	else {
		throw std::runtime_error("Type " + type + " doesn't exist.");
	}
}
