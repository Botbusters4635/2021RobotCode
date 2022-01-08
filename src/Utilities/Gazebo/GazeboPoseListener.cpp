//
// Created by abiel on 7/17/20.
//

#include "GazeboPoseListener.h"
#include <sstream>

GazeboPoseListener::GazeboPoseListener(const std::string &ntEntryName) :
		GazeboPoseListener(nt::NetworkTableInstance::GetDefault().GetEntry(ntEntryName)) {
	;
}

GazeboPoseListener::GazeboPoseListener(const nt::NetworkTableEntry &entry) {
	this->entry = entry;
}

bool GazeboPoseListener::hasNewValue() const {
	auto currentValue = entry.GetString("");
	
	if (previouslyReadValue.empty() and currentValue.empty())
		return false;
	
	return previouslyReadValue != currentValue;
}

RobotPose2D GazeboPoseListener::getPose() const {
	std::stringstream ss(entry.GetString(""));
	previouslyReadValue = ss.str();
	
	//X,Y,Theta
	std::vector<std::string> entries;
	std::string buf{};
	while (std::getline(ss, buf, ',')) {
		entries.emplace_back(buf);
	}
	
	if (entries.size() != 3) {
		return {0, 0, 0};
	}
	
	return {std::stod(entries.at(0)), std::stod(entries.at(1)), std::stod(entries.at(2))};
}