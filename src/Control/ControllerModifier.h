//
// Created by abiel on 9/15/19.
//

#ifndef ECTOCONTROL_CONTROLLERMODIFIER_H
#define ECTOCONTROL_CONTROLLERMODIFIER_H

#include "EctoControllerSource.h"
#include "EctoControllerOutput.h"

#include <functional>

class ControllerModifierSource : public EctoControllerSource {
public:
	ControllerModifierSource(EctoControllerSource &source, const std::function<double(double)> &positionFunction,
	                         const std::function<double(double)> &velocityFunction);
	
	double getPosition() const override;
	
	double getVelocity() const override;

private:
	std::function<double(double)> positionModFunction;
	std::function<double(double)> velocityModFunction;
	
	EctoControllerSource &baseSource;
};

class ControllerModifierOutput : public EctoControllerOutput {
public:
	ControllerModifierOutput(EctoControllerOutput &output, const std::function<double(double)> &function);
	
	void outputSet(double output) override;

private:
	std::function<double(double)> modifierFunction;
	EctoControllerOutput &baseOutput;
};


#endif //ECTOCONTROL_CONTROLLERMODIFIER_H
