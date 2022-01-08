//
// Created by Abiel on 9/11/18.
//

#include "Core/EctoInput/Buttons/ToggleButton.h"

ToggleButton::ToggleButton(bool defaultState) {
	this->currentStatus = defaultState;
}

bool ToggleButton::calculateOutput(bool input) {
	currentStatusMutex.lock();
	bool outValue = currentStatus;
	currentStatusMutex.unlock();
	
	if (input && input != previousStatus) {
		outValue = !outValue;
		
		currentStatusMutex.lock();
		currentStatus = outValue;
		currentStatusMutex.unlock();
	}
	
	previousStatusMutex.lock();
	this->previousStatus = input;
	previousStatusMutex.unlock();
	
	return outValue;
}