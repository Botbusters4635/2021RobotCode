//
// Created by abiel on 11/3/21.
//

#ifndef BOTBUSTERS_REBIRTH_FEEDFORWARDCONSTANT_H
#define BOTBUSTERS_REBIRTH_FEEDFORWARDCONSTANT_H

#include <cmath>

class FeedforwardConstant {
public:
	double kS{0}, kV{0}, kA{0};
	
	FeedforwardConstant(double kS, double kV, double kA) {
		this->kS = kS;
		this->kV = kV;
		this->kA = kA;
	}
	
	FeedforwardConstant() {
		;
	}
	
	
	double calcFF(double vel, double accel) {
		double ff = kV * vel;
		ff += kA * accel;
		
		ff += std::copysign(kS, ff);
		return ff;
	}
};

#endif //BOTBUSTERS_REBIRTH_FEEDFORWARDCONSTANT_H
