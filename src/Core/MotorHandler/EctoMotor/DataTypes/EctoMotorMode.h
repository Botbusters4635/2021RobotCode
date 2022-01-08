//
// Created by alberto on 31/07/19.
//

#ifndef ECTOCONTROL_ECTOMOTORMODE_H
#define ECTOCONTROL_ECTOMOTORMODE_H

#include <algorithm>
#include "EctoMotorType.h"

enum class MotorControlMode {
	Percent,
	Velocity,
	Position,
	MotionMagic,
	Current,
	Voltage
};

std::string toString(MotorControlMode mode);

MotorControlMode getControlModeFromString(const std::string &mode);

#endif //ECTOCONTROL_ECTOMOTORMODE_H
