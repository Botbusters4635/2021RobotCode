//
// Created by karen on 07/01/2020.
//

#ifndef BOTBUSTERS_REBIRTH_CONTROLPANELHANDLER_H
#define BOTBUSTERS_REBIRTH_CONTROLPANELHANDLER_H

#include <Core/EctoModule/System.h>
#include "EctoColor.h"
#include "rev/ColorSensorV3.h"
#include "rev/ColorMatch.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include "Core/MotorHandler/MotorManager.h"

enum class ControlPanelMode {
	Position,
	Rotation
};

class ControlPanelHandler : public System {
public:
	ControlPanelHandler(double motorVelocity = 0.1);
	
	void robotUpdate() override;
	
	void setTargetColor(EctoColor controlPanelTarget);
	
	void changeControlPanelMode(ControlPanelMode modeToChange);
	
	void setEnabled(bool enabled);

private:
	std::string gameDataString;
	
	EctoColor targetColor;
	EctoColor pastColor;
	
	double motorVelocity = 0;
	
	bool enabled = false;
	size_t count = 0;
	
	ControlPanelMode mode = ControlPanelMode::Rotation;
	
	rev::ColorSensorV3 colorSensor{frc::I2C::Port::kOnboard};
	rev::ColorMatch colorMatcher;
	
	const frc::Color kBlueTarget = frc::Color(0.140, 0.406, 0.452);
	const frc::Color kGreenTarget = frc::Color(0.181, 0.548, 0.269);
	const frc::Color kRedTarget = frc::Color(0.502, 0.346, 0.151);
	const frc::Color kYellowTarget = frc::Color(0.329, 0.533, 0.137);
	
	MotorManager &motorHandler = MotorManager::getInstance();
	std::shared_ptr<EctoMotor> controlPanelMotor;
	
	EctoColor getReadColor();
	
	static EctoColor getGameDataColor();
};

#endif //BOTBUSTERS_REBIRTH_CONTROLPANELHANDLER_H
