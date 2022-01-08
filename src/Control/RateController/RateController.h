//
// Created by alberto on 22/12/19.
//

#ifndef RATELIMITER_RATECONTROLLER_H
#define RATELIMITER_RATECONTROLLER_H

#include <chrono>

struct RateControllerConfig {
	/**
	 * Max rate that the variable is allowed to change at
	 */
	double maxRate = 0.0;
	/**
	 * Initial value of the variable
	 */
	double initialValue = 0.0;
	
	/**
	 * Max value allowed
	 */
	double maxValue = 0.0;
	
	/**
	 * Min value allowed
	 */
	double minValue = 0.0;
	
	/**
	 * Whether or not the variable should loopback to the min value when maxValue is reached, or vice versa
	 */
	bool continuous = false;
};

class RateController {
public:
	/**
	 * RateController allows a variable to be controled by a rate, using a multiplier of -1.0 to 1.0 that decides how much
	 * of the max rate is used to modify the variable.
	 * One use case of this class would be to have a target angle change with a controller, but with a max given rate.
	 * @param config Configuration struct that defines how the Rate Controller will behave
	 */
	explicit RateController(RateControllerConfig config);
	
	/**
	 * Change the variable by the maxRate * multiplier
	 * @param multiplier
	 */
	void changeValue(double multiplier);
	
	/**
	 * Change the variable by the maxRate * multiplier, with a given timeStep, useful for testing or using a system-wide timeStep
	 * @param multiplier
	 * @param timeStep
	 */
	void changeValue(double multiplier, double timeStep);
	
	/**
	 * Get the current value of the variable
	 * @return
	 */
	double get();

private:
	void changeValueBase(double multiplier, double timeStep);
	
	RateControllerConfig config;
	double currentRateMultiplier = 0.0;
	double currentValue = 0.0;
	bool lastTimeValid = false;
	bool clamped = false;
	
	std::chrono::high_resolution_clock::time_point lastTimeRan;
	
};


#endif //RATELIMITER_RATECONTROLLER_H
