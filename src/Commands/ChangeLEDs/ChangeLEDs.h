//
// Created by abiel on 2/19/20.
//

#ifndef BOTBUSTERSREBIRTH_CHANGELEDS_H
#define BOTBUSTERSREBIRTH_CHANGELEDS_H

#include <frc2/command/CommandHelper.h>
#include <frc2/command/CommandBase.h>

#include "Core/LEDManager/LedManager.h"

class ChangeLEDs : public frc2::CommandHelper<frc2::CommandBase, ChangeLEDs> {
public:
	ChangeLEDs(const PatternCommand &command, PatternPriority priority = PatternPriority::MedPriority);
	
	void Initialize() override;
	
	bool IsFinished() override;

private:
	PatternCommand command;
	PatternPriority priority;
	
	LEDManager &manager = LEDManager::getInstance();
};


#endif //BOTBUSTERSREBIRTH_CHANGELEDS_H
