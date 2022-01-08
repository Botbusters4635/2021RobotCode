//
// Created by abiel on 9/15/19.
//

#ifndef ECTOCONTROL_CONTROLLERCALLBACK_H
#define ECTOCONTROL_CONTROLLERCALLBACK_H

#include "EctoControllerOutput.h"
#include "EctoControllerSource.h"
#include <functional>

class ControllerCallbackSource : public EctoControllerSource {
public:
	ControllerCallbackSource(const std::function<double(void)> &positionFunction,
	                         const std::function<double(void)> &velocityFunction);
	
	double getPosition() const override;
	
	double getVelocity() const override;

private:
	std::function<double(void)> positionFunction;
	std::function<double(void)> velocityFunction;
};

class ControllerCallbackOutput : public EctoControllerOutput {
public:
	ControllerCallbackOutput(const std::function<void(double)> &function);
	
	void outputSet(double output) override;

private:
	std::function<void(double)> outputFunction;
};


#endif //ECTOCONTROL_CONTROLLERCALLBACK_H
