//
// Created by abiel on 7/16/20.
//

#include "GazeboPoseReset.h"

GazeboPoseReset::GazeboPoseReset(const std::string &gazeboPoseEntryName,
                                 const std::function<void(const RobotPose2D &)> &cb)
		: System("GazeboPoseReset") {
	table = ntInstance.GetTable(gazeboPoseEntryName);
	
	input.registerButton(resetButton, "A");
	
	this->cb = cb;
}

void GazeboPoseReset::robotUpdate() {
	if (resetButton.get() and resetButton.get() != lastResetButton) {
		log->info("Triggered reset!");
		RobotPose2D pose;
		pose.setX(table->GetNumber("x", 0));
		pose.setY(table->GetNumber("y", 0));
		pose.setTheta(table->GetNumber("theta", 0));
		cb(pose);
	}
	
	lastResetButton = resetButton.get();
}