//
// Created by abiel on 8/26/19.
//

#ifndef ECTOCONTROL_TANKMOTORVALUES_H
#define ECTOCONTROL_TANKMOTORVALUES_H

#include <algorithm>

class TankMotorValues {
public:
	TankMotorValues(double left, double right);
	
	TankMotorValues();
	
	double left, right;
	
	double getHighestVelocity() const;
	
	void clampVelocities(double maxVel);
	
	void normalizeVelocities();
};

#endif //ECTOCONTROL_TANKMOTORVALUES_H
