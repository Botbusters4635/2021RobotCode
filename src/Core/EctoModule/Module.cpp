//
// Created by hiram on 3/07/19.
//
#include "Module.h"

Module::Module(const std::string &name) : name(name) {
	log = spdlog::stdout_color_mt(name);
	log->info("Starting...");
}

std::string Module::getName() {
	return name;
}
