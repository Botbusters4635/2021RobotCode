//
// Created by alberto on 02/09/19.
//

#include <stdexcept>
#include <cmath>
#include "Control/EctoSMC/EctoSMC.h"
#include <iostream>

EctoSMC::EctoSMC(EctoControllerSource &source, EctoControllerOutput &output, const EctoSMCConfig &config) : source(
		source), output(output) {
	setConfig(config);
}

void EctoSMC::update() {
	double kADelta = 0.0;
	double slidingSurfaceResult = getSlidingSurface();
	
	if (ka > config.kMin) {
		kADelta = config.k3 * sign(std::abs(slidingSurfaceResult) - config.mu);
	} else {
		kADelta = config.kMin;
	}
	
	ka += kADelta;
	
	output.outputSet(ka * std::sqrt(std::abs(slidingSurfaceResult)) * sign(slidingSurfaceResult) +
	                 config.k2 * slidingSurfaceResult);
}

void EctoSMC::setSetpoint(double setpoint) {
	this->setpoint = setpoint;
}

double EctoSMC::getSetpoint() const {
	return setpoint;
}

double EctoSMC::getError() const {
	return error;
}

double EctoSMC::getSlidingSurface() {
	error = setpoint - source.getPosition();
	
	if (config.continous && std::abs(error) > maxError) {
		int loops = (int) std::floor(std::abs(error / (maxError * 2.0))) + 1;
		error -= std::copysign((maxError * 2.0) * loops, error);
	}
	
	double errorDelta = error - lastError;
	lastError = error;
	
	return config.lambda * error + errorDelta;
}


void EctoSMC::setConfig(const EctoSMCConfig &config) {
	this->config = config;
	
	error = 0;
	lastError = 0;
	ka = 0;
	
	maxError = (config.maxInput - config.minInput) / 2;
}

const EctoControllerOutput &EctoSMC::getControllerOutput() const {
	return output;
}

const EctoControllerSource &EctoSMC::getControllerSource() const {
	return source;
}