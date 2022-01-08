//
// Created by Abiel on 8/26/19.
//

#ifndef ECTOCONTROL_SWERVEWHEEL_H
#define ECTOCONTROL_SWERVEWHEEL_H

#include "Math/DataTypes/Point2D.h"
#include <iostream>

class SwerveWheel {
public:
	double wheelVelocity = 0.0;
	
	double wheelAngle = 0.0;
	double wheelAngularVelocity = 0.0;
	
	double wheelPosition = 0.0;
	
	void invertValue();
	
	static SwerveWheel invertValue_copy(const SwerveWheel &value); //Returns rotated value by 180
	
	static double getAngleBetween(const SwerveWheel &current, const SwerveWheel &target);
	
	bool operator==(const SwerveWheel &rhs) const;
	
	friend std::ostream &operator<<(std::ostream &os, const SwerveWheel &wheel);
	
	void roundToEpsilon();
	
	static SwerveWheel roundToEpsilon_copy(const SwerveWheel &wheel);

private:
	constexpr static double epsilon = 0.0001;
};

#endif //ECTOCONTROL_SWERVEWHEEL_H
