//
// Created by Abiel on 1/14/19.
//

#include "Control/MotionProfiles/MotionProfile.h"
#include <cmath>

MotionProfile::MotionProfile(const MotionProfileConfig &config) {
	this->initialPosition = config.initialPosition;
	this->finalPosition = config.finalPosition;
	
	this->initialVelocity = config.initialVelocity;
	
	this->maximumVelocity = config.maximumVelocity;
	this->maximumAcceleration = config.maximumAcceleration;
	
	this->startTime = config.startTime;
	
	throwDistance = finalPosition - initialPosition;
}

double MotionProfile::getStartTime() const {
	return startTime;
}

bool MotionProfileConfig::operator==(const MotionProfileConfig &other) const {
	bool equal = false;
	
	equal = this->initialPosition == other.initialPosition;
	equal = equal and this->finalPosition == other.finalPosition;
	equal = equal and this->startTime == other.startTime;
	equal = equal and this->initialVelocity == other.initialVelocity;
	equal = equal and this->maximumVelocity == other.maximumVelocity;
	return equal and this->maximumAcceleration == other.maximumAcceleration;
}

bool MotionProfileConfig::operator!=(const MotionProfileConfig &other) const {
	return !(*this == other);
}

std::ostream &operator<<(std::ostream &os, const MotionProfileConfig &config) {
	return os << config.initialPosition << ',' << config.finalPosition << ',' <<
	          config.startTime << ',' << config.initialVelocity << ',' << config.maximumVelocity << ',' <<
	          config.maximumAcceleration;
}

std::istream &operator>>(std::istream &in, MotionProfileConfig &config) {
	in >> config.startTime;
	in.ignore();
	in >> config.finalPosition;
	in.ignore();
	in >> config.startTime;
	in.ignore();
	in >> config.initialVelocity;
	in.ignore();
	in >> config.maximumVelocity;
	in.ignore();
	in >> config.maximumAcceleration;
	
	return in;
}