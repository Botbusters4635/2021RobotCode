//
// Created by abiel on 2/8/20.
//

#include "ManagerHandler.h"

void ManagerHandler::addManager(const std::function<void()> &updateFunction) {
	managerUpdate.emplace_back(updateFunction);
}

void ManagerHandler::robotUpdate() {
	for (auto updateFunction: managerUpdate) {
		updateFunction();
	}
}