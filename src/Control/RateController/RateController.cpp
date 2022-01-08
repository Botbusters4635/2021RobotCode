//
// Created by alberto on 22/12/19.
//

#include "Control/RateController/RateController.h"
#include <stdexcept>
#include <cmath>
#include <iostream>

RateController::RateController(RateControllerConfig config) {
	if (config.maxRate <= 0.0) {
		throw std::invalid_argument(
				"Max rate cannot be less than or equal zero, given value: " + std::to_string(config.maxRate));
	}
	
	currentValue = config.initialValue;
	
	if (config.maxValue != 0 or config.minValue != 0) {
		if (config.minValue >= config.maxValue) {
			throw std::invalid_argument(
					"Min value is greater than max value, given values: Min " + std::to_string(config.minValue) +
					" Max " + std::to_string(config.maxValue));
		}
		
		clamped = true;
		
		if (currentValue < config.minValue) {
			currentValue = config.minValue;
		}
		if (currentValue > config.maxValue) {
			currentValue = config.maxValue;
		}
	}
	
	this->config = config;
}

void RateController::changeValue(double multiplier) {
	std::chrono::high_resolution_clock::time_point currentTime = std::chrono::high_resolution_clock::now();
	if (!lastTimeValid) {
		lastTimeRan = currentTime;
		lastTimeValid = true;
		return;
	}
	
	double timeStep = std::chrono::duration<double>(currentTime - lastTimeRan).count();
	
	lastTimeRan = currentTime;
	
	changeValue(multiplier, timeStep);
}

void RateController::changeValue(double multiplier, double timeStep) {
	currentRateMultiplier = std::abs(multiplier) <= 1.0 ? multiplier : std::copysign(1.0, multiplier);
	
	double delta = multiplier * config.maxRate * timeStep;
	
	if (clamped && !config.continuous) {
		if (currentValue + delta < config.minValue) {
			delta = -currentValue + config.minValue;
			
		} else if (currentValue + delta > config.maxValue) {
			delta = -currentValue + config.maxValue;
		}
	}
	
	currentValue += delta;
	if (config.continuous) {
		if (currentValue < config.minValue) {
			currentValue += config.maxValue - config.minValue;
		} else if (currentValue > config.maxValue) {
			currentValue -= config.maxValue - config.minValue;
		}
	}
}

double RateController::get() {
	return currentValue;
}