//
// Created by alberto on 13/07/19.
//

#ifndef ECTOCONTROL_SIMPLEPIDOUTPUT_H
#define ECTOCONTROL_SIMPLEPIDOUTPUT_H

#include "Control/EctoControllerOutput.h"

class SimpleControllerOutput : public EctoControllerOutput {
public:
	double get() {
		return value;
	}
	
	void set(double value) {
		outputSet(value);
	}
	
	void outputSet(double set) override {
		this->value = set;
	}

private:
	double value = 0;
};


#endif //ECTOCONTROL_SIMPLEPIDOUTPUT_H
