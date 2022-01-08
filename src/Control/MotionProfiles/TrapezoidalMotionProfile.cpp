//
// Created by Abiel on 1/29/19.
//

#include "Control/MotionProfiles/TrapezoidalMotionProfile.h"
#include <cmath>

TrapezoidalMotionProfile::TrapezoidalMotionProfile(const MotionProfileConfig &config) : MotionProfile(
		config) {
	if (throwDistance < 0.0) {
		isInverted = true;
		throwDistance = std::abs(throwDistance);
		initialVelocity = -initialVelocity;
	}
	
	timeToAccelerate = (maximumVelocity - initialVelocity) / maximumAcceleration;
	distanceToAccelerate = std::abs(
			timeToAccelerate * initialVelocity + .5 * maximumAcceleration * std::pow(timeToAccelerate, 2.0));
	
	timeToDeccelToZero = (maximumVelocity) / maximumAcceleration;
	distanceToDeccelerateToZero =
			timeToDeccelToZero * maximumVelocity + .5 * -maximumAcceleration * std::pow(timeToDeccelToZero, 2.0);
	
	distanceToCruise = throwDistance - distanceToAccelerate - distanceToDeccelerateToZero;
	timeToCruise = distanceToCruise / maximumVelocity;
	
	if (distanceToAccelerate + distanceToDeccelerateToZero > throwDistance) {
		//https://math.stackexchange.com/questions/637042/calculate-maximum-velocity-given-accel-decel-initial-v-final-position
		//Time to accelerate for
		timeToAccelerate = -(initialVelocity / maximumAcceleration) + (1 / maximumAcceleration) * std::sqrt(
				(maximumAcceleration * std::pow(initialVelocity, 2.0) +
				 2.0 * std::pow(maximumAcceleration, 2.0) * throwDistance) / (maximumAcceleration * 2.0));
		
		//Time to deccel for
		timeToDeccelToZero = ((initialVelocity + maximumAcceleration * timeToAccelerate) / maximumAcceleration);
		
		maximumVelocity = initialVelocity + maximumAcceleration * timeToAccelerate;
		
		distanceToAccelerate =
				timeToAccelerate * initialVelocity + .5 * maximumAcceleration * std::pow(timeToAccelerate, 2.0);
		distanceToDeccelerateToZero =
				timeToDeccelToZero * maximumVelocity + .5 * -maximumAcceleration * std::pow(timeToDeccelToZero, 2.0);
		
		isTriangular = true;
		timeToCruise = 0.0;
		distanceToCruise = 0.0;
		
	}
	
	totalTime = timeToAccelerate + timeToCruise + timeToDeccelToZero;
	
	
}

EctoMotionProfileStatus TrapezoidalMotionProfile::getCurrentStatus_time(double time) const {
	if (time < 0.0)
		throw std::runtime_error("Invalid time");
	
	if (time < timeToAccelerate)
		return EctoMotionProfileStatus::accelerating;
	
	if (time < timeToAccelerate + timeToCruise and !isTriangular)
		return EctoMotionProfileStatus::cruise;
	
	if (time < timeToAccelerate + timeToCruise + timeToDeccelToZero)
		return EctoMotionProfileStatus::deaccelerating;
	
	return EctoMotionProfileStatus::stopped;
}

EctoMotionProfileStatus TrapezoidalMotionProfile::getCurrentStatus_distance(double distance) const {
	if (distance < distanceToAccelerate)
		return EctoMotionProfileStatus::accelerating;
	
	if (distance < distanceToAccelerate + distanceToCruise and !isTriangular)
		return EctoMotionProfileStatus::cruise;
	
	if (distance < distanceToAccelerate + distanceToCruise + distanceToDeccelerateToZero)
		return EctoMotionProfileStatus::deaccelerating;
	
	return EctoMotionProfileStatus::stopped;
}

double TrapezoidalMotionProfile::getPosition_time(double time) const {
	time -= startTime;
	double output = 0.0;
	
	switch (getCurrentStatus_time(time)) {
		case EctoMotionProfileStatus::accelerating:
			output = initialVelocity * time + .5 * maximumAcceleration * std::pow(time, 2.0) + initialPosition;
			break;
		
		case EctoMotionProfileStatus::cruise:
			time -= timeToAccelerate;
			output = (time) * maximumVelocity + distanceToAccelerate + initialPosition;
			break;
		
		case EctoMotionProfileStatus::deaccelerating:
			time -= (timeToAccelerate + timeToCruise);
			output = maximumVelocity * time + .5 * -maximumAcceleration * std::pow(time, 2.0) + distanceToCruise +
			         distanceToAccelerate + initialPosition;
			break;
		
		case EctoMotionProfileStatus::stopped:
			output = (distanceToCruise + distanceToDeccelerateToZero + distanceToAccelerate) + initialPosition;
			break;
		
		default:
			output = 0.0;
			break;
	}
	if (isInverted) {
		return initialPosition - (output - initialPosition);
	} else {
		return output;
	}
}

double TrapezoidalMotionProfile::getVelocity_time(double time) const {
	time -= startTime;
	double output = 0.0;
	
	switch (getCurrentStatus_time(time)) {
		case EctoMotionProfileStatus::accelerating:
			output = initialVelocity + (maximumAcceleration * time);
			break;
		case EctoMotionProfileStatus::cruise:
			output = maximumVelocity;
			break;
		
		case EctoMotionProfileStatus::deaccelerating:
			output = maximumVelocity - (maximumAcceleration * (time - timeToAccelerate - timeToCruise));
			break;
		
		case EctoMotionProfileStatus::stopped:
			output = 0.0;
			break;
		
		default:
			output = 0.0;
			break;
	}
	
	return !isInverted ? output : -output;
}

double TrapezoidalMotionProfile::getAcceleration_time(double time) const {
	time -= startTime;
	
	switch (getCurrentStatus_time(time)) {
		case EctoMotionProfileStatus::accelerating:
			return maximumAcceleration;
		
		case EctoMotionProfileStatus::cruise:
			return 0.0;
		
		case EctoMotionProfileStatus::deaccelerating:
			return -maximumAcceleration;
		
		case EctoMotionProfileStatus::stopped:
			return 0.0;
		
		default:
			return 0.0;
	}
}

double TrapezoidalMotionProfile::getPosition_distance(double distance) const {
	//Why would you use this
	return distance;
}

double TrapezoidalMotionProfile::getVelocity_distance(double distance) const {
	distance -= initialPosition;
	
	double sgn = std::copysign(1.0, maximumVelocity - initialVelocity);
	double output = 0.0;
	
	switch (getCurrentStatus_distance(distance)) {
		case EctoMotionProfileStatus::accelerating:
			output = std::sqrt(std::pow(initialVelocity, 2.0) + 2.0 * maximumAcceleration * distance) * sgn;
			break;
		
		case EctoMotionProfileStatus::cruise:
			output = maximumVelocity;
			break;
		
		case EctoMotionProfileStatus::deaccelerating:
			distance -= distanceToAccelerate + distanceToCruise;
			output = std::sqrt(std::pow(maximumVelocity, 2.0) + 2.0 * -maximumAcceleration * distance);
			break;
		
		case EctoMotionProfileStatus::stopped:
			output = 0.0;
			break;
		
		default:
			output = 0.0;
			break;
	}
	return !isInverted ? output : -output;
	
}

double TrapezoidalMotionProfile::getAcceleration_distance(double distance) const {
	double output = 0.0;
	
	switch (getCurrentStatus_distance(distance)) {
		case EctoMotionProfileStatus::accelerating:
			output = maximumAcceleration;
			break;
		
		case EctoMotionProfileStatus::cruise:
			output = 0;
			break;
		
		case EctoMotionProfileStatus::deaccelerating:
			output = -maximumAcceleration;
			break;
		
		case EctoMotionProfileStatus::stopped:
			output = 0.0;
			break;
		
		default:
			output = 0.0;
			break;
	}
	
	return !isInverted ? output : -output;
}


bool TrapezoidalMotionProfile::isDone(double time) {
	return time < getTimeDuration();
}

double TrapezoidalMotionProfile::getTimeDuration() const {
	return totalTime;
}

