//
// Created by Abiel on 9/11/18.
//

#include "Core/EctoInput/Axis/JoystickAxis.h"

void JoystickAxis::updateValue(double value) {
	std::lock_guard<std::mutex> lock(joystickMutex);
	this->outValue = calculateOutput(value);
}

double JoystickAxis::calculateOutput(double value) {
	return value;
}

double JoystickAxis::get() const {
	std::lock_guard<std::mutex> lock(joystickMutex);
	return outValue;
}