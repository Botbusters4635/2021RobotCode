//
// Created by Abiel on 1/29/19.
//

#ifndef ECTOJERKTESTS_TRAPEZOIDALMOTIONPROFILEREBIRTH_H
#define ECTOJERKTESTS_TRAPEZOIDALMOTIONPROFILEREBIRTH_H

#include "MotionProfile.h"

class TrapezoidalMotionProfile : public MotionProfile {
public:
	explicit TrapezoidalMotionProfile(const MotionProfileConfig &config);
	
	bool isDone(double time);
	
	double getPosition_time(double time) const override;
	
	double getVelocity_time(double time) const override;
	
	double getAcceleration_time(double time) const override;
	
	double getPosition_distance(double distance) const;
	
	double getVelocity_distance(double distance) const;
	
	double getAcceleration_distance(double distance) const;
	
	double getTimeDuration() const override;

private:
	EctoMotionProfileStatus getCurrentStatus_time(double time) const;
	
	EctoMotionProfileStatus getCurrentStatus_distance(double distance) const;
	
	double timeToAccelerate;
	double distanceToAccelerate;
	
	double timeToCruise;
	double distanceToCruise;
	
	double timeToDeccelToZero;
	double distanceToDeccelerateToZero;
	
	double totalTime;
	bool isInverted = false;
	bool isTriangular = false;
};


#endif //ECTOJERKTESTS_TRAPEZOIDALMOTIONPROFILEREBIRTH_H
