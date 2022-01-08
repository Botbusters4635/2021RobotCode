//
// Created by abiel on 1/2/20.
//

#ifndef BOTBUSTERSREBIRTH_ECTOSWERVEINPUTHANDLER_H
#define BOTBUSTERSREBIRTH_ECTOSWERVEINPUTHANDLER_H

#include "EctoSwerve.h"
#include "Core/EctoModule/System.h"
#include <Core/EctoInput/InputManager.h>
#include <Core/EctoInput/Buttons/EctoButton.h>
#include <Core/EctoInput/Buttons/ToggleButton.h>
#include <Core/EctoInput/Axis/JoystickAxisExpo.h>

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>

#include <frc/SlewRateLimiter.h>
#include <frc/PIDController.h>

#include <units/velocity.h>
#include <units/angular_velocity.h>

class EctoSwerveInputHandler : public System {
public:
	EctoSwerveInputHandler(EctoSwerve &swerveIn);
	
	void robotInit() override;
	
	void robotUpdate() override;

private:
	EctoSwerve &swerve;
	
	InputManager &input = InputManager::getInstance();
	
	Twist2D output;
	
	/**
	 * NT
	 */
	nt::NetworkTableInstance ntInstance = nt::NetworkTableInstance::GetDefault();
	std::shared_ptr<nt::NetworkTable> table, visionTable;
	nt::NetworkTableEntry visionAngle, hasTarget;
	
	const double joystickExpo = 0.15;
	const double joystickDeadzone = 0.2;
	
	const double triggerExpo = 0.2;
	const double triggerDeadzone = 0.2;
	
	JoystickAxisExpo strafeAxis{joystickExpo, joystickDeadzone}, forwardAxis{joystickExpo,
	                                                                         joystickDeadzone}, rotationAxis{
			joystickExpo, 0.25};
	JoystickAxisExpo brakeTrigger{triggerExpo, triggerDeadzone};
	
	const double slowModeReduction = 0.9;
	const double minimumBrakeValue = 0.15;
	
	EctoButton resetYaw, fastMode, testPivotPoint, visionButton, disableFieldOrientated;
	
	const bool filterInputs = true;
	units::meters_per_second_t maximumInputAcceleration{30.0}; // / 1s
	units::radians_per_second_t maximumInputAngularAcceleration{M_PI * 8}; // / 1S
	
	frc::SlewRateLimiter<units::meters_per_second> xFilter{maximumInputAcceleration / units::second_t(1.0)};
	frc::SlewRateLimiter<units::meters_per_second> yFilter{maximumInputAcceleration / units::second_t(1.0)};
	frc::SlewRateLimiter<units::radians_per_second> thetaFilter{maximumInputAngularAcceleration / units::second_t(1.0)};
	double headingTarget = 0.0;
	frc2::PIDController thetaPID{3.5, 0.0, 0.015};
	
	frc2::PIDController visionAnglePID{0.022, 0, 0.00095};
	
	double lastTime;
};


#endif //BOTBUSTERSREBIRTH_ECTOSWERVEINPUTHANDLER_H
