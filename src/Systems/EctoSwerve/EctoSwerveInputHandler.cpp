//
// Created by abiel on 1/2/20.
//

#include "EctoSwerveInputHandler.h"

EctoSwerveInputHandler::EctoSwerveInputHandler(EctoSwerve &swerveIn) : System("EctoSwerveRebirthInputHandler"),
                                                                       swerve(swerveIn) {
	input.registerAxis(strafeAxis, "rightX");
	input.registerAxis(forwardAxis, "rightY");
	input.registerAxis(rotationAxis, "leftX");
	
	input.registerAxis(brakeTrigger, "leftTrigger");
	
	input.registerButton(fastMode, "leftJoystick");
	//input.registerButton(&fieldOrientedEnable, "rightJoy");
	
	input.registerButton(resetYaw, "select");
	input.registerButton(visionButton, "rightBumper");
	input.registerButton(disableFieldOrientated, "leftBumper");
	
	//input.registerButton(testPivotPoint, "rightBumper");
	
	thetaPID.EnableContinuousInput(-M_PI, M_PI);
	
	table = ntInstance.GetTable("EctoSwerveInputHandler");
	visionTable = ntInstance.GetTable("photonvision/gloworm");
	visionAngle = visionTable->GetEntry("targetYaw");
	hasTarget = visionTable->GetEntry("hasTarget");
}

void EctoSwerveInputHandler::robotInit() {
	;
}

void EctoSwerveInputHandler::robotUpdate() {
	const double dt = frc::Timer::GetFPGATimestamp() - lastTime;
	
	Twist2D joystickInput(-forwardAxis.get(), -strafeAxis.get(), -rotationAxis.get());
	//log->info("{},{}", forwardAxis.get(), strafeAxis.get());
	frc::SmartDashboard::PutNumber("Debug/FowardAxis", forwardAxis.get());
	frc::SmartDashboard::PutNumber("Debug/StrafeAxis", -strafeAxis.get());
	frc::SmartDashboard::PutNumber("Debug/RotationAxis", rotationAxis.get());
	
	table->GetEntry("RotationAxis").SetDouble(-rotationAxis.get());
	
	output = joystickInput;
	headingTarget += -rotationAxis.get() * M_PI / 2 * dt;
	
	if (headingTarget > M_PI) {
		headingTarget -= M_PI * 2;
	} else if (headingTarget < -M_PI) {
		headingTarget += M_PI * 2;
	}
	
	table->GetEntry("HeadingTarget").SetDouble(headingTarget);
	
	thetaPID.SetSetpoint(headingTarget);
	//output *= 4.4;
	output.setDtheta(output.getDtheta() * 0.75);
	//output.setDtheta(thetaPID.Calculate(swerve.getYaw()));
	
	if (!fastMode.get()) {
		output *= slowModeReduction;
	}
	
	if (std::abs(brakeTrigger.get()) > 0) {
		const double reductionFactor = std::max((1.0 - brakeTrigger.get()), minimumBrakeValue);
		//output *= reductionFactor;
	}
	
	if (resetYaw.get()) {
		swerve.zeroYaw();
		headingTarget = 0.0;
	}
	
	
	if (filterInputs) {
		output.setDx(xFilter.Calculate(units::meters_per_second_t(output.getDx())).to<double>());
		output.setDy(yFilter.Calculate(units::meters_per_second_t(output.getDy())).to<double>());
		output.setDtheta(thetaFilter.Calculate(units::radians_per_second_t(output.getDtheta())).to<double>());
		//output.setDtheta(thetaPID.Calculate(swerve.getYaw()));
	}
	
	frc::Translation2d centerOfRotation(units::meter_t(0), units::meter_t(0));
	frc::Rotation2d gyroRot(units::radian_t(swerve.getYaw()));
	
	if (testPivotPoint.get()) {
		centerOfRotation = frc::Translation2d(units::meter_t(-1.1), units::meter_t(1.1));
		centerOfRotation = centerOfRotation.RotateBy(gyroRot);
	}
	
	if (visionButton.get() and hasTarget.GetBoolean(false)) {
		double setpoint = 0;
		double state = visionAngle.GetDouble(0); //TODO add network tables
		double visionOut = visionAnglePID.Calculate(state, setpoint);
		output.setDtheta(visionOut);
	}
	
	frc::ChassisSpeeds chassisSpeeds;
	if (disableFieldOrientated.get()) {
		chassisSpeeds.vx = units::meters_per_second_t(output.getDx());
		chassisSpeeds.vy = -units::meters_per_second_t(output.getDy());
		chassisSpeeds.omega = units::radians_per_second_t(output.getDtheta());
		
		swerve.setPercent(chassisSpeeds);
		
		
	} else {
		auto chassisSpeeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(units::meters_per_second_t(output.getDx()),
		                                                                 units::meters_per_second_t(output.getDy()),
		                                                                 units::radians_per_second_t(
				                                                                 output.getDtheta()),
		                                                                 frc::Rotation2d(
				                                                                 units::radian_t(swerve.getYaw())));
		chassisSpeeds.vy *= -1;
		swerve.setPercent(chassisSpeeds);
	}




//     chassisSpeeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(units::meters_per_second_t(output.getDx()),
//                                                                     units::meters_per_second_t(output.getDy()),
//                                                                     units::radians_per_second_t(output.getDtheta()),
//                                                                     frc::Rotation2d(units::radian_t(swerve.getYaw())));
//    chassisSpeeds.vy *= -1;
//    swerve.setPercent(chassisSpeeds);
	
	
	
	//chassisSpeeds.vx = units::meters_per_second_t (0);
	//chassisSpeeds.vy = units::meters_per_second_t (0);
	//chassisSpeeds.omega = units::radians_per_second_t(output.getDtheta());
	//log->info("{}",output.getDtheta());Entry/X
	
	
}
