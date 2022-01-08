//
// Created by Abiel on 9/11/18.
//

#ifndef BOTBUSTERSREBIRTH_JOYSTICKAXIS_H
#define BOTBUSTERSREBIRTH_JOYSTICKAXIS_H

#include <mutex>

class JoystickAxis {
public:
	/**
	 * Pass the raw joystick axis here
	 * @param value
	 */
	void updateValue(double value);
	
	double get() const;

protected:
	virtual double calculateOutput(double value);
	
	double outValue = 0.0;
	
	mutable std::mutex joystickMutex;
};


#endif //BOTBUSTERSREBIRTH_JOYSTICKAXIS_H
