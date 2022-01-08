//
// Created by Abiel on 1/9/19.
//

#include "Control/MotionProfiles/JerkVelocityControl.h"
#include <cmath>

JerkVelocityControl::JerkVelocityControl(const JerkVelocityControlConfig &config) {
	this->config = config;
	previousLoopTime = std::chrono::high_resolution_clock::now();
}

double JerkVelocityControl::update(double currentVelocity, double targetVelocity) {
	std::chrono::high_resolution_clock::time_point startTime = std::chrono::high_resolution_clock::now();
	double timeStep = std::chrono::duration<double>(startTime - previousLoopTime).count();
	
	bool isDeaccelerating = targetVelocity < currentVelocity;
	
	//Change in needed to change velocity to 0
	double velocityNeeded = std::pow(previousAcceleration, 2) / (2.0 * config.setJerk);
	
	double setAcceleration = 0.0;
	
	if (std::abs(targetVelocity - currentVelocity) > velocityNeeded) {
		//Proceed normally
		if (isDeaccelerating) {
			setAcceleration = previousAcceleration - (config.setJerk / timeStep);
		} else {
			setAcceleration = previousAcceleration + (config.setJerk / timeStep);
		}
	} else {
		//Reduce acceleration to 0
		//setAcceleration = 0;
		setAcceleration = previousAcceleration - (config.setJerk / timeStep);
	}
	
	if (std::abs(setAcceleration) < config.accelerationTreshold) {
		setAcceleration = 0;
	}
	
	if (std::abs(setAcceleration) > config.maximumAcceleration) {
		setAcceleration = std::copysign(config.maximumAcceleration, setAcceleration);
	}
	
	double setVelocity = currentVelocity + (setAcceleration / timeStep);
	
	if (std::abs(setVelocity) > config.maximumVelocity) {
		setVelocity = std::copysign(config.maximumVelocity, setVelocity);
	}
	
	/*
	bool withinTarget = std::abs(currentVelocity - targetVelocity) < config.treshold;
	bool isDeaccelerating = targetVelocity < currentVelocity;

	double setAcceleration;

	if (withinTarget) {
		//setAcceleration = (targetVelocity - currentVelocity) / timeStep;

		//Limit to jerk
//        if(std::abs(setAcceleration - previousAcceleration) > config.setJerk){
//            if (isDeaccelerating) {
//                setAcceleration = previousAcceleration - (config.setJerk * timeStep);
//            } else {
//                setAcceleration = previousAcceleration + (config.setJerk * timeStep);
//            }
//        }
	} else {
		if (isDeaccelerating) {
			setAcceleration = previousAcceleration - (config.setJerk * timeStep);
		} else {
			setAcceleration = previousAcceleration + (config.setJerk * timeStep);
		}
	}

	if (std::abs(setAcceleration) > config.maximumAcceleration) {
		setAcceleration = std::copysign(config.maximumAcceleration, setAcceleration);
	}

	std::cout << setAcceleration << std::endl;
	std::cout << std::endl;

	double setVelocity = currentVelocity + (setAcceleration * timeStep);

	if (std::abs(setVelocity) > config.maximumVelocity) {
		setVelocity = std::copysign(config.maximumVelocity, setVelocity);
	}
	 */
	
	previousVelocity = currentVelocity;
	previousAcceleration = setAcceleration;
	previousLoopTime = startTime;
	
	return setVelocity;
}