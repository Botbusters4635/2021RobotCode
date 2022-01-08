//
// Created by abiel on 7/15/20.
//

#include "EctoTankInputHandler.h"

EctoTankInputHandler::EctoTankInputHandler(const std::shared_ptr<EctoTank> &tank,
                                           const EctoTankInputHandlerConfig &config)
		: System("EctoTankInputHandler") {
	if (tank == nullptr)
		throw std::runtime_error("EctoTankInputHandler given nullptr as EctoTank");
	
	this->tank = tank;
	this->config = config;
}

void EctoTankInputHandler::robotInit() {
	input.registerAxis(xRightJoy, rightXJoyName);
	input.registerAxis(xLeftJoy, leftXJoyName);
}

void EctoTankInputHandler::robotUpdate() {
	auto rightPercentage = xRightJoy.get();
	auto leftPercentage = xLeftJoy.get();
	
	tank->setPercentage(leftPercentage, rightPercentage);
}