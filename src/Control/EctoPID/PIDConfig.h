//
// Created by alberto on 13/07/19.
//

#ifndef MOTORTRANSLATOR_PIDCONFIG_H
#define MOTORTRANSLATOR_PIDCONFIG_H


class PIDConfig {
public:
	double p = 0;
	double i = 0;
	double d = 0;
	double f = 0;
	
	double iZone = 0;
	double maxAbsoluteIntegral = -1;
	
	bool continous = false;
	
	double maxInput = 1;
	double minInput = -1;
	
	bool clamped = false;
	
	double maxOutput = 0;
	double minOutput = 0;
	
	double deadband = 0;
	
	bool operator==(const PIDConfig &rhs) const {
		return
				(p == rhs.p and
				 i == rhs.i and
				 d == rhs.d and
				 f == rhs.f and
				
				 iZone == rhs.iZone and
				 maxAbsoluteIntegral == rhs.maxAbsoluteIntegral and
				 continous == rhs.continous and
				
				 maxInput == rhs.maxInput and
				 minInput == rhs.minInput and
				
				 clamped == rhs.clamped and
				
				 maxOutput == rhs.maxOutput and
				 minOutput == rhs.minOutput and
				
				 deadband == rhs.deadband);
	}
	
	bool operator!=(const PIDConfig &rhs) const {
		return !(*this == rhs);
	}
};


#endif //MOTORTRANSLATOR_PIDCONFIG_H
