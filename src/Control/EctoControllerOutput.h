//
// Created by alberto on 13/07/19.
//

#ifndef MOTORTRANSLATOR_ECTOPIDOUTPUT_H
#define MOTORTRANSLATOR_ECTOPIDOUTPUT_H


class EctoControllerOutput {
public:
	virtual void outputSet(double set) = 0;
};


#endif //MOTORTRANSLATOR_ECTOPIDOUTPUT_H
