//
// Created by Abiel on 9/11/18.
//

#include "EctoButton.h"

void EctoButton::updateStatus(bool status) {
	std::lock_guard<std::mutex> lock(buttonMutex);
	outValue = calculateOutput(status);
}

bool EctoButton::calculateOutput(bool input) {
	return input;
}

bool EctoButton::get() const {
	std::lock_guard<std::mutex> lock(buttonMutex);
	return outValue;
}