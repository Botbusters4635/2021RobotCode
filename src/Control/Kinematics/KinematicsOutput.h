//
// Created by alberto on 15/08/19.
//

#ifndef ECTOCONTROL_KINEMATICSOUTPUT_H
#define ECTOCONTROL_KINEMATICSOUTPUT_H

template<class T>
class KinematicsOutput {
public:
	virtual T getCurrentState() = 0;
	
	virtual void writeMotors(const T &values) = 0;
};

#endif //ECTOCONTROL_KINEMATICSOUTPUT_H
