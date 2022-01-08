//
// Created by Abiel on 9/11/18.
//

#ifndef BOTBUSTERSREBIRTH_TOGGLEBUTTON_H
#define BOTBUSTERSREBIRTH_TOGGLEBUTTON_H

#include "EctoButton.h"

class ToggleButton : public EctoButton {
public:
	ToggleButton(bool defaultState = false);

protected:
	bool calculateOutput(bool input) override;

private:
	std::mutex previousStatusMutex;
	bool previousStatus = false;
	
	std::mutex currentStatusMutex;
	bool currentStatus = false;
};


#endif //BOTBUSTERSREBIRTH_TOGGLEBUTTON_H
