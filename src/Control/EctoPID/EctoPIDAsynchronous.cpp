//
// Created by abiel on 1/4/20.
//

#include "Control/EctoPID/EctoPIDAsynchronous.h"

EctoPIDAsynchronous::EctoPIDAsynchronous(const PIDConfig &config) : pidController(source, output, config) {
	;
}

void EctoPIDAsynchronous::setConfig(const PIDConfig &config) {
	pidController.setConfig(config);
}

void EctoPIDAsynchronous::setSetpoint(double setpoint) {
	pidController.setSetpoint(setpoint);
}

double EctoPIDAsynchronous::update(double input) {
	source.setPosition(input);
	pidController.update();
	return output.get();
}