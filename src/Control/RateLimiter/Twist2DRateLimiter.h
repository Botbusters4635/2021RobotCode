//
// Created by abiel on 1/28/20.
//

#ifndef ECTOCONTROL_TWIST2DRATELIMITER_H
#define ECTOCONTROL_TWIST2DRATELIMITER_H

#include <Math/DataTypes/Twist2D.h>
#include "Control/RateLimiter/RateLimiter.h"
#include <memory>

/**
 * Limits a twist 2D
 */
class Twist2DRateLimiter {
public:
	Twist2DRateLimiter(double maxRate, bool limitX = true, bool limitY = true, bool limitTheta = true);
	
	void set(const Twist2D &target);
	
	Twist2D get(double timeStep);

private:
	std::unique_ptr<RateLimiter> xLimiter, yLimiter, thetaLimiter;
	
	bool limitX, limitY, limitTheta;
	
	Twist2D setValue;
};


#endif //ECTOCONTROL_TWIST2DRATELIMITER_H
