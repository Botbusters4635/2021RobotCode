//
// Created by alberto on 13/07/19.
//

#ifndef MOTORTRANSLATOR_ECTOPIDSOURCE_H
#define MOTORTRANSLATOR_ECTOPIDSOURCE_H


class EctoControllerSource {
public:
	virtual double getPosition() const = 0;
	
	virtual double getVelocity() const = 0;
};


#endif //MOTORTRANSLATOR_ECTOPIDSOURCE_H
