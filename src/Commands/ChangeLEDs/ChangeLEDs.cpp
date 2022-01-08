//
// Created by abiel on 2/19/20.
//

#include "ChangeLEDs.h"

ChangeLEDs::ChangeLEDs(const PatternCommand &command, PatternPriority priority) {
	this->command = command;
	this->priority = priority;
	this->SetName("ChangeLEDs");
}

void ChangeLEDs::Initialize() {
	manager.queueCommand(command, priority);
}

bool ChangeLEDs::IsFinished() {
	return true;
}