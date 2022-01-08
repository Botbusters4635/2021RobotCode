//
// Created by abiel on 11/3/21.
//

#ifndef BOTBUSTERS_REBIRTH_HOLONOMICFEEDFORWARD_H
#define BOTBUSTERS_REBIRTH_HOLONOMICFEEDFORWARD_H

#include "FeedforwardConstant.h"
#include <frc/drive/Vector2d.h>
#include <functional>

class HolonomicFeedforward {
public:
	HolonomicFeedforward(const FeedforwardConstant &forwardConstant, const FeedforwardConstant &strafeConstant) {
		this->forward = forwardConstant;
		this->strafe = strafeConstant;
	}
	
	HolonomicFeedforward() {
		;
	}
	
	frc::Vector2d calculateFF(const frc::Vector2d &vel, const frc::Vector2d &accel) const {
		double forwardFF = forward.kV * vel.x;
		forwardFF += forward.kA * accel.x;
		
		double strafeFF = strafe.kV * vel.y;
		strafeFF += strafe.kA * accel.y;
		
		frc::Vector2d ff(forwardFF, strafeFF);
		double ffMag = ff.Magnitude();
		if (std::abs(ffMag) < 0.001) {
			return {0, 0};
		}
		
		frc::Vector2d ffUnit(ff.x / ffMag, ff.y / ffMag);
		
		ff.x += std::copysign(ffUnit.x * forward.kS, ff.x);
		ff.y += std::copysign(ffUnit.y * strafe.kS, ff.y);
//        ff.y *= -1;
		
		return ff;
	}

private:
	FeedforwardConstant forward, strafe;
};

#endif //BOTBUSTERS_REBIRTH_HOLONOMICFEEDFORWARD_H
