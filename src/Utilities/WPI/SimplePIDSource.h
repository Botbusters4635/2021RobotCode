//
// Created by abiel on 1/8/20.
//

#ifndef BOTBUSTERSREBIRTH_SIMPLEPIDSOURCE_H
#define BOTBUSTERSREBIRTH_SIMPLEPIDSOURCE_H

#include <frc/PIDSource.h>

class SimplePIDSource : public frc::PIDSource {
public:
	double PIDGet() override {
		return value;
	};
	
	void set(double value) {
		this->value = value;
	}

private:
	double value;
	
};


#endif //BOTBUSTERSREBIRTH_SIMPLEPIDSOURCE_H
