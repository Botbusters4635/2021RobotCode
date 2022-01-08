//
// Created by ajahueym on 1/13/21.
//

#ifndef BOTBUSTERS_REBIRTH_DEFAULTMAPPINGS_H
#define BOTBUSTERS_REBIRTH_DEFAULTMAPPINGS_H

#include "InputManager.h"

namespace EctoInput {
	struct DefaultMappings {
		static InputManager::JoystickMap Xbox() {
			InputManager::JoystickMap xbox;
			xbox.buttonsMapping = {
					{"A",             1},
					{"B",             2},
					{"X",             3},
					{"Y",             4},
					{"leftBumper",    5},
					{"rightBumper",   6},
					{"select",        8},
					{"start",         7},
					{"home",          9},
					{"leftJoystick",  10},
					{"rightJoystick", 11}
			};
			
			xbox.axesMapping = {
					{"leftX",        0},
					{"leftY",        1},
					{"leftTrigger",  2},
					{"rightTrigger", 3},
					{"rightX",       4},
					{"rightY",       5}
			};
			
			return xbox;
		}
		
	};
}
#endif //BOTBUSTERS_REBIRTH_DEFAULTMAPPINGS_H
