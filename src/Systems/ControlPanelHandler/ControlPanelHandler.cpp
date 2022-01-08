//
// Created by karen on 07/01/2020.
//

#include "ControlPanelHandler.h"
#include <frc/DriverStation.h>

ControlPanelHandler::ControlPanelHandler(double motorVelocity) : System("ControlPanelHandler") {
	colorMatcher.AddColorMatch(kBlueTarget);
	colorMatcher.AddColorMatch(kGreenTarget);
	colorMatcher.AddColorMatch(kRedTarget);
	colorMatcher.AddColorMatch(kYellowTarget);
	
	controlPanelMotor = motorHandler.getMotor("controlPanelMotor");
	controlPanelMotor->setOpenLoopRampRate(2.0);
	
	this->motorVelocity = motorVelocity;
}

EctoColor ControlPanelHandler::getReadColor() {
	double confidence = 0.0;
	frc::Color readColor = colorSensor.GetColor();
	std::optional<frc::Color> matchedColor = colorMatcher.MatchColor(readColor, confidence);
	
	frc::SmartDashboard::PutNumber("Red", readColor.red);
	frc::SmartDashboard::PutNumber("Green", readColor.green);
	frc::SmartDashboard::PutNumber("Blue", readColor.blue);
	
	frc::SmartDashboard::PutNumber("Confidence", confidence);
	
	if (matchedColor) {
		if (matchedColor == kBlueTarget) {
			return EctoColor::Blue;
		} else if (matchedColor == kRedTarget) {
			return EctoColor::Red;
		} else if (matchedColor == kGreenTarget) {
			return EctoColor::Green;
		} else if (matchedColor == kYellowTarget) {
			return EctoColor::Yellow;
		}
	}
	
	return EctoColor::Error;
}

void ControlPanelHandler::setTargetColor(EctoColor controlPanelTarget) {
	targetColor = offsetColor(controlPanelTarget, 2);
//	switch (controlPanelTarget) {
//		case EctoColor::Red:
//			targetColor = EctoColor::Blue;
//			break;
//		case EctoColor::Blue:
//			targetColor = EctoColor::Red;
//			break;
//		case EctoColor::Green:
//			targetColor = EctoColor::Yellow;
//			break;
//		case EctoColor::Yellow:
//			targetColor = EctoColor::Green;
//			break;
//		default:
//			targetColor = EctoColor::Error;
//			break;
//	}
}

void ControlPanelHandler::changeControlPanelMode(ControlPanelMode modeToChange) {
	mode = modeToChange;
}

void ControlPanelHandler::setEnabled(bool enabled) {
	this->enabled = enabled;
}

void ControlPanelHandler::robotUpdate() {
	if (!enabled) {
		controlPanelMotor->set(0.0);
		return;
	}
	
	EctoColor detectedColor = getReadColor();
	
	switch (mode) {
		case ControlPanelMode::Position:
			if (detectedColor != EctoColor::Error) {
				double colorError = targetColor - detectedColor;
				if (std::abs(colorError) > 2) {
					colorError += std::copysign(4, -colorError);
				}
				
				double output = std::copysign(motorVelocity, colorError);
				
				if (colorError == 0) {
					output = 0;
				}
				
				controlPanelMotor->set(output);
			}
			break;
		
		case ControlPanelMode::Rotation:
			if (count <= 24) {
				if (pastColor != detectedColor and detectedColor != EctoColor::Error) {
					count++;
					controlPanelMotor->set(motorVelocity);
					pastColor = detectedColor;
					frc::SmartDashboard::PutNumber("ControlPanel/RotationCount", count);
				}
			} else {
				controlPanelMotor->set(0.0);
			}
			break;
	}
}

EctoColor ControlPanelHandler::getGameDataColor() {
	std::string gameData;
	gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();
	if (gameData.length() > 0) {
		switch (gameData[0]) {
			case 'B' :
				return EctoColor::Blue;
				break;
			case 'G' :
				return EctoColor::Green;
				break;
			case 'R' :
				return EctoColor::Red;
				break;
			case 'Y' :
				return EctoColor::Yellow;
				break;
			default :
				return EctoColor::Error;
				break;
		}
	} else {
		return EctoColor::Error;
	}
}
    

       

