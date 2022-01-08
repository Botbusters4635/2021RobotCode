//
// Created by Abiel on 1/9/19.
//

#ifndef BOTBUSTERSREBIRTH_JERKVELOCITYCONTROL_H
#define BOTBUSTERSREBIRTH_JERKVELOCITYCONTROL_H

#include <chrono>

struct JerkVelocityControlConfig {
	double setJerk;
	double maximumAcceleration;
	double maximumVelocity;
	
	double velocityTreshold;
	double accelerationTreshold;
};

class JerkVelocityControl {
public:
	JerkVelocityControl(const JerkVelocityControlConfig &config);
	
	double update(double currentVelocity, double targetVelocity);
	
	double previousVelocity;
	double previousAcceleration;
private:
	JerkVelocityControlConfig config;
	
	std::chrono::system_clock::time_point previousLoopTime;
};


#endif //BOTBUSTERSREBIRTH_JERKVELOCITYCONTROL_H
