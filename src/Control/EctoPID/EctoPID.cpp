//
// Created by alberto on 13/07/19.
//

#include <iostream>
#include <cmath>
#include "Control/EctoPID/EctoPID.h"

EctoPID::EctoPID(EctoControllerSource &source, EctoControllerOutput &output, const PIDConfig &config) : source(source),
                                                                                                        output(output) {
	setConfig(config);
}

void EctoPID::update() {
	if (!initialTimeSet) {
		lastTimeUpdate = std::chrono::high_resolution_clock::now();
		initialTimeSet = true;
	}
	
	std::lock_guard<std::mutex> lock(configMutex);
	double timeStep = std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - lastTimeUpdate).count();
	double sourceValue = source.getPosition();
	
	setPointMutex.lock();
	rawError = setpoint - sourceValue;
	setPointMutex.unlock();
	
	error = rawError;
	
	if (config.continous && std::abs(error) > maxError) {
		int loops = std::floor(std::abs(error / (maxError * 2.0))) + 1;
		error -= std::copysign((maxError * 2.0) * loops, error);
	}
	errorDelta = (error - lastError) / timeStep;
	
	if (std::abs(error) >= config.iZone) {
		errorIntegral += error * timeStep;
	} else {
		errorIntegral = 0;
	}
	
	if (std::abs(errorIntegral) > config.maxAbsoluteIntegral && config.maxAbsoluteIntegral != -1) {
		errorIntegral = std::copysign(config.maxAbsoluteIntegral, errorIntegral);
	}
	
	double outputValue = (config.p * error) + (config.i * errorIntegral) + (config.d * errorDelta) + config.f;
	
	if (config.clamped && outputValue > config.maxOutput) {
		outputValue = std::copysign(config.maxOutput, outputValue);
	}
	
	if (std::abs(outputValue) < config.deadband) {
		outputValue = 0.0;
	}
	
	output.outputSet(outputValue);
	
	lastError = error;
	lastTimeUpdate = std::chrono::high_resolution_clock::now();
	
	
}

void EctoPID::setSetpoint(double setpoint) {
	setPointMutex.lock();
	this->setpoint = setpoint;
	setPointMutex.unlock();
}

double EctoPID::getSetpoint() const {
	std::lock_guard<std::mutex> lock(setPointMutex);
	return setpoint;
}

double EctoPID::getError() const {
	return error;
}

double EctoPID::getRawError() const {
	return rawError;
}

double EctoPID::getErrorIntegral() const {
	return errorIntegral;
}

double EctoPID::getErrorDelta() const {
	return errorDelta;
}

double EctoPID::getMaxError() const {
	return maxError;
}

const PIDConfig &EctoPID::getConfig() const {
	std::lock_guard<std::mutex> lock(configMutex);
	return config;
}

void EctoPID::setConfig(const PIDConfig &config) {
	if (config.continous) {
		if (config.minInput > config.maxInput) {
			throw std::invalid_argument("EctoPID: config.minInput is greater than config.maxInput");
		}
		maxError = (config.maxInput - config.minInput) / 2;
	}
	
	if (config.deadband < 0) {
		throw std::invalid_argument("EctoPID: config.deadband is less than zero, should be an absolute value");
	}
	
	if (config.clamped) {
		if (config.minOutput > config.maxOutput) {
			throw std::invalid_argument("EctoPID: config.minOutput is greater than config.maxOutput");
		}
		if (config.deadband > config.maxOutput) {
			throw std::invalid_argument("EctoPID: config.deadband is greater than config.maxOutput");
		}
	}
	if (config.maxAbsoluteIntegral != -1 && config.maxAbsoluteIntegral < 0) {
		throw std::invalid_argument(
				"EctoPID: config.maxAbsoluteIntegral is less than zero, should be an absolute value");
	}
	
	if (config.iZone < 0) {
		throw std::invalid_argument("EctoPID: config.iZone is less than zero, should be an absolute value");
	}
	
	std::lock_guard<std::mutex> lock(configMutex);
	this->config = config;
}

const EctoControllerOutput &EctoPID::getControllerOutput() const {
	return output;
}

const EctoControllerSource &EctoPID::getControllerSource() const {
	return source;
}
