//
// Created by abiel on 1/4/20.
//

#ifndef ECTOCONTROL_ECTOPIDASYNCHRONOUS_H
#define ECTOCONTROL_ECTOPIDASYNCHRONOUS_H

#include "Control/EctoPID/EctoPID.h"
#include "Control/SimpleControllerOutput.h"
#include "Control/SimpleControllerSource.h"

class EctoPIDAsynchronous {
public:
	EctoPIDAsynchronous(const PIDConfig &config);
	
	void setSetpoint(double setpoint);
	
	void setConfig(const PIDConfig &config);
	
	double update(double input);

private:
	EctoPID pidController;
	SimpleControllerOutput output;
	SimpleControllerSource source;
};


#endif //ECTOCONTROL_ECTOPIDASYNCHRONOUS_H
