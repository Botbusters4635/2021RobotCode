//
// Created by abiel on 9/10/19.
//

#ifndef ECTOCONTROL_ECTOCONTROLLER_H
#define ECTOCONTROL_ECTOCONTROLLER_H

#include "Control/EctoControllerOutput.h"
#include "Control/EctoControllerSource.h"

/**
 * Generic class for all (hopefully) Ecto- Controllers
 */

class EctoController {
public:
	virtual void setSetpoint(double setpoint) = 0;
	
	virtual double getSetpoint() const = 0;
	
	virtual double getError() const = 0;
	
	virtual void update() = 0;
	
	virtual const EctoControllerOutput &getControllerOutput() const = 0;
	
	virtual const EctoControllerSource &getControllerSource() const = 0;
};

#endif //ECTOCONTROL_ECTOCONTROLLER_H
