//
// Created by hiram on 28/06/19.
//

#ifndef BOTBUSTERS_REBIRTH_SYSTEM_H
#define BOTBUSTERS_REBIRTH_SYSTEM_H


#include <vector>
#include <memory>
#include "Module.h"


class System : public Module {
public:
	System(const std::string &name);
	
	virtual void robotInit();
	
	virtual void robotUpdate();
	
	virtual void disabledInit();
	
	virtual void disabledUpdate();
	
};


#endif //BOTBUSTERS_REBIRTH_SYSTEM_H
