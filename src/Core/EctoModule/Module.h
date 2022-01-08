//
// Created by hiram on 3/07/19.
//

#ifndef BOTBUSTERS_REBIRTH_MODULE_H
#define BOTBUSTERS_REBIRTH_MODULE_H

#include <memory>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>

// Base class for all subsystems.

class Module {
public:
	Module(const std::string &name);
	
	std::string getName();

protected:
	std::shared_ptr<spdlog::logger> log;

private:
	std::string name;
	
};


#endif //BOTBUSTERS_REBIRTH_MODULE_H
