//
// Created by Abiel on 1/14/19.
//

#ifndef BOTBUSTERSREBIRTH_MOTIONPROFILE_H
#define BOTBUSTERSREBIRTH_MOTIONPROFILE_H

#include <iostream>

class MotionProfileConfig {
public:
	double initialPosition = 0.0;
	double finalPosition = 0.0;
	
	double startTime = 0.0;
	
	double initialVelocity = 0.0;
	
	double maximumVelocity = 0.0;
	double maximumAcceleration = 0.0;
	
	friend std::ostream &operator<<(std::ostream &os, MotionProfileConfig const &point);
	
	friend std::istream &operator>>(std::istream &in, MotionProfileConfig &point);
	
	bool operator==(const MotionProfileConfig &other) const;
	
	bool operator!=(const MotionProfileConfig &other) const;
};

enum class EctoMotionProfileStatus {
	accelerating,
	cruise,
	deaccelerating,
	stopped
};

class MotionProfile {
public:
	explicit MotionProfile(const MotionProfileConfig &config);
	
	MotionProfile() = delete;
	
	virtual double getAcceleration_time(double currentTime) const = 0;
	
	virtual double getVelocity_time(double currentTime) const = 0;
	
	virtual double getPosition_time(double currentTime) const = 0;
	
	double getStartTime() const;
	
	virtual double getTimeDuration() const = 0;

protected:
	double initialVelocity, initialPosition, finalPosition, maximumVelocity, maximumAcceleration;
	double startTime;
	double throwDistance;
};


#endif //BOTBUSTERSREBIRTH_MOTIONPROFILE_H
