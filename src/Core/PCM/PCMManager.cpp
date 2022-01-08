//
// Created by abiel on 8/22/19.
//

#include "PCMManager.h"

PCMManager::PCMManager() : Manager("PCMManager") {
	log->info("Initializing PCMManager...");
	
}

void PCMManager::init() {
	initializePistons();
}

void PCMManager::setPCMConfig(const std::list<PistonInfo> &pistonInfo) {
	this->pistonInfo = pistonInfo;
}

void PCMManager::initializePistons() {
	for (const auto &info: pistonInfo) {
		auto solenoid = std::make_shared<frc::DoubleSolenoid>(info.id1, info.id2);
		if (pistons.find(info.name) != pistons.end()) log->error("Duplicate piston name!");
		pistons[info.name] = solenoid;
	}
	hasInit = true;
}

std::shared_ptr<frc::DoubleSolenoid> PCMManager::getPiston(const std::string &name) {
	if (!hasInit)
		throw std::runtime_error(
				"PCMManager has not been initialized, check if EctoRobot::getPistonConfig has been overriden");
	
	auto it = pistons.find(name);
	
	if (it == pistons.end()) {
		throw std::runtime_error(fmt::format("Invalid piston name: {}", name));
	}
	
	return it->second;
}

void PCMManager::update() {
	;
}