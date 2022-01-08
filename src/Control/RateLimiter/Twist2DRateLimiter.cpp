//
// Created by abiel on 1/28/20.
//

#include "Control/RateLimiter/Twist2DRateLimiter.h"

Twist2DRateLimiter::Twist2DRateLimiter(double maxRate, bool limitX, bool limitY, bool limitTheta) {
	this->limitX = limitX;
	this->limitY = limitY;
	this->limitTheta = limitTheta;
	
	if (limitX) {
		xLimiter = std::make_unique<RateLimiter>(maxRate);
	}
	
	if (limitY) {
		yLimiter = std::make_unique<RateLimiter>(maxRate);
	}
	
	if (limitTheta) {
		thetaLimiter = std::make_unique<RateLimiter>(maxRate);
	}
}

void Twist2DRateLimiter::set(const Twist2D &target) {
	if (limitX) {
		xLimiter->set(target.getDx());
	}
	
	if (limitY) {
		yLimiter->set(target.getDy());
	}
	
	if (limitTheta) {
		thetaLimiter->set(target.getDtheta());
	}
	
	setValue = target;
}

Twist2D Twist2DRateLimiter::get(double timeStep) {
	Twist2D output = setValue;
	
	if (limitX) {
		output.setDx(xLimiter->get(timeStep));
	}
	
	if (limitY) {
		output.setDy(yLimiter->get(timeStep));
	}
	
	if (limitTheta) {
		output.setDtheta(thetaLimiter->get(timeStep));
	}
	
	return output;
}