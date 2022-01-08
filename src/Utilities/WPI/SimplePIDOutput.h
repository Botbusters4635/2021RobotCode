//
// Created by abiel on 1/8/20.
//

#ifndef BOTBUSTERSREBIRTH_SIMPLEPIDOUTPUT_H
#define BOTBUSTERSREBIRTH_SIMPLEPIDOUTPUT_H

#include <frc/PIDOutput.h>

class SimplePIDOutput : public frc::PIDOutput {
public:
	void PIDWrite(double output) override {
		value = output;
	};
	
	double get() const {
		return value;
	};

private:
	double value;
};


#endif //BOTBUSTERSREBIRTH_SIMPLEPIDOUTPUT_H
