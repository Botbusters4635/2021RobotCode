//
// Created by Abiel on 9/11/18.
//

#include "Core/EctoInput/Axis/JoystickAxisExpo.h"
#include <cmath>

JoystickAxisExpo::JoystickAxisExpo(double expoValue, double deadZone) {
	this->expoValue = expoValue;
	this->deadZone = deadZone;
	
	if (this->deadZone < 0 || this->deadZone > MAX_VALUE) {
		throw std::runtime_error(
				"Invalid deadZone given to JoystickAxisExpo, needs to be between 0 and " + std::to_string(MAX_VALUE));
	}
	
	if (this->expoValue < 0 || this->expoValue > MAX_VALUE) {
		throw std::runtime_error(
				"Invalid expoValue given to JoystickAxisExpo, needs to be between 0 and " + std::to_string(MAX_VALUE));
	}
	
	this->slope = MAX_VALUE / (MAX_VALUE - deadZone);
	this->offset = -(deadZone * this->slope);
}

double JoystickAxisExpo::applyDeadzone(double value) {
	if (std::abs(value) >= deadZone) {
		return value;
	}
	return 0;
}


double JoystickAxisExpo::calculateOutput(double value) {
	double tempValue = applyDeadzone(value);
	double linearizedValue = 0.0;
	if (tempValue > 0) {
		linearizedValue = tempValue * slope + offset;
	} else if (tempValue < 0) {
		linearizedValue = tempValue * slope - offset;
	}
	
	return expoValue * std::pow(linearizedValue, 3.0) + (1.0 - expoValue) * linearizedValue;
}