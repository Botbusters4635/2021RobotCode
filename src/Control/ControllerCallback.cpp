//
// Created by abiel on 9/15/19.
//

#include "ControllerCallback.h"

ControllerCallbackSource::ControllerCallbackSource(const std::function<double(void)> &positionFunction,
                                                   const std::function<double(void)> &velocityFunction) {
	this->positionFunction = positionFunction;
	this->velocityFunction = velocityFunction;
}

double ControllerCallbackSource::getVelocity() const {
	return velocityFunction();
}

double ControllerCallbackSource::getPosition() const {
	return positionFunction();
}

ControllerCallbackOutput::ControllerCallbackOutput(const std::function<void(double)> &function) {
	this->outputFunction = function;
}

void ControllerCallbackOutput::outputSet(double output) {
	outputFunction(output);
}