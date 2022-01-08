//
// Created by alberto on 22/12/19.
//

#include "Control/RateLimiter/RateLimiter.h"
#include <cmath>
#include <stdexcept>

RateLimiter::RateLimiter(double maxRate, double initialValue) {
	currentValue = initialValue;
	targetValue = initialValue;
	
	if (maxRate <= 0.0) {
		throw std::logic_error(
				"Max rate cannot be less than or equal zero for RateLimiter, given value= " + std::to_string(maxRate));
	}
	
	this->maxRate = maxRate;
}

void RateLimiter::setMaxRate(double rate) {
	this->maxRate = rate;
}

void RateLimiter::set(double target) {
	this->targetValue = target;
}

double RateLimiter::get(double timeStep) {
	double error = targetValue - currentValue;
	double maxAllowedChange = maxRate * timeStep;
	
	if (std::abs(error) < maxAllowedChange) {
		currentValue = targetValue;
	} else {
		currentValue += std::copysign(maxAllowedChange, error);
	}
	
	return currentValue;
}


double RateLimiter::get() {
	std::chrono::high_resolution_clock::time_point currentTime = std::chrono::high_resolution_clock::now();
	
	if (!lastTimeValid) {
		lastTimeRan = currentTime;
		lastTimeValid = true;
		return currentValue;
	}
	
	double timeStep = std::chrono::duration<double>(currentTime - lastTimeRan).count();
	
	lastTimeRan = currentTime;
	
	return get(timeStep);
}