//
// Created by abiel on 9/15/19.
//

#include "ControllerModifier.h"

ControllerModifierSource::ControllerModifierSource(EctoControllerSource &source,
                                                   const std::function<double(double)> &positionFunction,
                                                   const std::function<double(double)> &velocityFunction) : baseSource(
		source) {
	positionModFunction = positionFunction;
	velocityModFunction = velocityFunction;
}

double ControllerModifierSource::getPosition() const {
	return positionModFunction(baseSource.getPosition());
}

double ControllerModifierSource::getVelocity() const {
	return positionModFunction(baseSource.getVelocity());
}

ControllerModifierOutput::ControllerModifierOutput(EctoControllerOutput &output,
                                                   const std::function<double(double)> &function) : baseOutput(output) {
	modifierFunction = function;
}

void ControllerModifierOutput::outputSet(double output) {
	baseOutput.outputSet(modifierFunction(output));
}