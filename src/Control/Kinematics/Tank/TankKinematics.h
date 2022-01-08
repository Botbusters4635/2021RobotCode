//
// Created by abiel on 8/26/19.
//

#ifndef ECTOCONTROL_TANKKINEMATICS_H
#define ECTOCONTROL_TANKKINEMATICS_H

#include "TankMotorValues.h"
#include "Math/DataTypes/Twist2D.h"

class TankKinematics {
public:
	TankKinematics(double length, double width);
	
	TankMotorValues calculateKinematics(const Twist2D &targetVelocity) const;
	
	Twist2D calculateInverseKinematics(const TankMotorValues &currentValues) const;

private:
	double length, width;
};

#endif //ECTOCONTROL_TANKKINEMATICS_H
