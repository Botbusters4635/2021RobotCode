//
// Created by alberto on 22/12/19.
//

#ifndef ECTOCONTROLLER_RATELIMITER_H
#define ECTOCONTROLLER_RATELIMITER_H

#include <chrono>

class RateLimiter {
public:
	/**
	 * Rate limiter is a helper class that prevents a variable from changing faster than the desired rate, useful for
	 * easily implementing ramp rates.
	 * @param maxRate Max rate of change per second allowed
	 * @param initialValue Initial value of variable
	 */
	explicit RateLimiter(double maxRate, double initialValue = 0.0);
	
	/**
	 * Get the current rate limited value of the variable
	 * @return
	 */
	double get();
	
	/**
	 * Get the current rate limited value, with a given timeStep, useful for testing or using a system-wide timeStep
	 * @param timeStep
	 * @return
	 */
	double get(double timeStep);
	
	/**
	 * set the current target for the rate limited variable
	 * @param target
	 * @return
	 */
	void set(double target);
	
	void setMaxRate(double rate);

private:
	double maxRate;
	double currentValue;
	double targetValue;
	
	bool lastTimeValid = false;
	std::chrono::high_resolution_clock::time_point lastTimeRan;
	
};


#endif //ECTOCONTROLLER_RATELIMITER_H
