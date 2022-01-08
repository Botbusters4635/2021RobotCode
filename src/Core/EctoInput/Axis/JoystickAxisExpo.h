//
// Created by Abiel on 9/11/18.
//

#ifndef BOTBUSTERSREBIRTH_JOYSTICKAXISEXPO_H
#define BOTBUSTERSREBIRTH_JOYSTICKAXISEXPO_H

#include "JoystickAxis.h"

class JoystickAxisExpo : public JoystickAxis {
public:
	JoystickAxisExpo(double expoValue, double deadZone);

protected:
	double calculateOutput(double value) override;
	
	double applyDeadzone(double value);

private:
	double expoValue = 1.0;
	double deadZone = 0.2;
	double slope = 0.0;
	double offset = 0.0;
	
	const double MAX_VALUE = 1.0;
};


#endif //BOTBUSTERSREBIRTH_JOYSTICKAXISEXPO_H
