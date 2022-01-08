//
// Created by alberto on 13/07/19.
//

#ifndef ECTOCONTROL_SIMPLEPIDSOURCE_H
#define ECTOCONTROL_SIMPLEPIDSOURCE_H

#include "Control/EctoControllerSource.h"

class SimpleControllerSource : public EctoControllerSource {
public:
	void setPosition(double position) {
		this->position = position;
	}
	
	double getPosition() const override {
		return position;
	}
	
	void setVelocity(double velocity) {
		this->velocity = velocity;
	}
	
	double getVelocity() const override {
		return velocity;
	}

private:
	double position = 0;
	double velocity = 0;
};


#endif //ECTOCONTROL_SIMPLEPIDSOURCE_H
