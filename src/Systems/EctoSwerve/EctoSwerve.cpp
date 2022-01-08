//
// Created by abiel on 1/2/20.
//

#include "EctoSwerve.h"
#include "frc/smartdashboard/SmartDashboard.h"

EctoSwerve::EctoSwerve(const EctoSwerveConfig &config) : System("EctoSwerve") {
	/*
	 * Kinematics init
	 */
	kinematics = createKinematics(units::meter_t(config.length), units::meter_t(config.width));
	
	/*
	 * Odometry init
	 */
	odometry = std::make_unique<frc::SwerveDriveOdometry<4>>(
			*kinematics, frc::Rotation2d(0, 0), frc::Pose2d()
	);
	
	PIDConfig steerConfig, wheelConfig;
	steerConfig.p = 0.27;
	steerConfig.i = 0;
	steerConfig.d = 0.00175;
	wheelConfig.p = 0.00039;
	wheelConfig.i = 0;
	wheelConfig.d = 0.00006;
	
	SwerveModuleConfig moduleConfig;
	moduleConfig.gearRatio = config.gearRatio;
	moduleConfig.wheelCircumference = config.wheelCircumference;
	moduleConfig.steerPID = steerConfig;
	moduleConfig.wheelPID = wheelConfig;
	
	std::vector<std::string> motorPrefixes = {"front_left", "front_right", "back_left", "back_right"};
//    std::vector<double> analogOffsets = {2.66 + M_PI, -1.32, 0.78 + M_PI, 1.54}; //fl, fr, bl, br
	std::vector<double> analogOffsets = {-2.08, -2.80, -0.99, 1.38}; //fl, fr, bl, br
//    std::vector<double> analogOffsets = {-1.75 + M_PI, -1,0.31  + M_PI, -2.08}; //fl, fr, bl, br
	
	for (size_t i = 0; i < modules.size(); i++) {
		const auto prefix = motorPrefixes[i];
		auto steerName = fmt::format("{}_steer", prefix);
		auto wheelName = fmt::format("{}_wheel", prefix);
		auto steerMotor = motorHandler.getMotor(steerName);
		auto wheelMotor = motorHandler.getMotor(wheelName);
		moduleConfig.analogOffset = analogOffsets[i];
		
		modules[i] = std::make_unique<SwerveModule>(prefix, steerMotor, wheelMotor, moduleConfig);
	}
	steerPIDNT = std::make_unique<NetworkTablePID>("EctoSwerve/SteerPID", moduleConfig.steerPID,
	                                               [&](const auto &config) {
		                                               for (auto &module: modules) {
			                                               module->setSteerPID(config);
		                                               }
	                                               });
	
	wheelPIDNT = std::make_unique<NetworkTablePID>("EctoSwerve/WheelPID", moduleConfig.wheelPID,
	                                               [&](const auto &config) {
		                                               for (auto &module: modules) {
			                                               module->setWheelPID(config);
		                                               }
	                                               });
	
	table = ntInstance.GetTable("EctoSwerve");

#ifndef SIMULATION
	/**
	* Gyro init
	*/
	adis = std::make_unique<frc::ADIS16470_IMU>(frc::ADIS16470_IMU::IMUAxis::kZ, frc::SPI::Port::kOnboardCS0,
	                                            frc::ADIS16470CalibrationTime::_4s);
	//adis.Calibrate();
	
	adis->SetYawAxis(frc::ADIS16470_IMU::IMUAxis::kZ);
#endif
	
	
	zeroYaw();
	initNetworkTables();
}

void EctoSwerve::initNetworkTables() {
	;
}

void EctoSwerve::updateNetworkTables() {
	table->GetEntry("EctoSwerve/CurrentHeading").SetDouble(getYaw());
	
	auto currentPose = getPose();
	
	table->GetEntry("EctoSwerve/Odometry/Velocity/dX").SetDouble(currentVelocity.vx.value());
	table->GetEntry("EctoSwerve/Odometry/Velocity/dY").SetDouble(currentVelocity.vy.value());
	table->GetEntry("EctoSwerve/Odometry/Velocity/dTheta").SetDouble(currentVelocity.omega.value());
	
	table->GetEntry("EctoSwerve/Odometry/X").SetDouble(currentPose.X().value());
	table->GetEntry("EctoSwerve/Odometry/Y").SetDouble(currentPose.Y().value());
	table->GetEntry("EctoSwerve/Odometry/Heading").SetDouble(currentPose.Rotation().Degrees().value());
}

void EctoSwerve::setVelocity(const frc::ChassisSpeeds &target) {
	auto states = SwerveState(kinematics->ToSwerveModuleStates(target));
	SwerveState::optimizeValues(SwerveState(), states, SwerveState(), lastState, 0);
	setModules(states, MotorControlMode::Velocity); //TODO normalize to maximum velocity
	lastState = states;
}

void EctoSwerve::setPercent(const frc::ChassisSpeeds &target, const Point2D &point) {
	frc::Translation2d cor(units::meter_t(point.getX()), units::meter_t(point.getY()));
	auto wpiStates = kinematics->ToSwerveModuleStates(target, cor);
	kinematics->NormalizeWheelSpeeds(&wpiStates, units::meters_per_second_t(0.85));
	
	auto states = SwerveState(wpiStates);
	SwerveState::optimizeValues(SwerveState(), states, SwerveState(), lastState, 0);
	
	setModules(states, MotorControlMode::Percent);
	lastState = states;
}

void EctoSwerve::setVoltage(const frc::ChassisSpeeds &target) {
	auto wpiStates = kinematics->ToSwerveModuleStates(target);
	kinematics->NormalizeWheelSpeeds(&wpiStates, units::meters_per_second_t(12.0));
	auto states = SwerveState(wpiStates);
	SwerveState::optimizeValues(SwerveState(), states, SwerveState(), lastState, 0);
	
	setModules(states, MotorControlMode::Voltage);
	lastState = states;
}

SwerveState EctoSwerve::getMotorStates() const {
	SwerveState values;
	
	for (int i = 0; i < 4; i++) {
		*values.wheels[i] = modules[i]->getState();
	}
	
	return values;
}

void EctoSwerve::setModules(const SwerveState &rawSetpoint, MotorControlMode controlMode) {
	auto state = getMotorStates();
	auto dt = frc::Timer::GetFPGATimestamp() - lastRunTime;
	
	auto setpoint = SwerveState::optimizeValues(state, rawSetpoint, lastState, lastSetpoint, dt);
	
	for (int i = 0; i < 4; i++) {
		auto wheel = setpoint.wheels[i];
		modules[i]->setState(wheel->wheelAngle, wheel->wheelVelocity, controlMode);
	}
	
	lastSetpoint = setpoint;
	lastState = state;
	lastRunTime = frc::Timer::GetFPGATimestamp();
}

void EctoSwerve::robotInit() {
	;
}

void EctoSwerve::robotUpdate() {
	updateNetworkTables();
	
	auto motorStates = getMotorStates();
	updateOdometry(motorStates);
}

void EctoSwerve::zeroYaw() {
	/**
	 * NavX Reset
	 */
	headingZero = getYaw(false);
}

void EctoSwerve::setYaw(double yaw) {
#ifndef SIMULATION
	throw std::runtime_error("Tried to set yaw in a real robot! This function is intended only for sim");
#endif
	simYaw = yaw;
}

double EctoSwerve::getYaw(bool useZero) const {
	double yaw = 0;

#ifndef SIMULATION
	yaw = adis->GetAngle() * (M_PI / 180.0);
	yaw = EctoMath::wrapAngle(yaw);
#else
	yaw = EctoMath::wrapAngle(simYaw);
#endif
	
	if (useZero) {
		yaw -= headingZero;
	}
	
	yaw = EctoMath::wrapAngle(yaw);
	
	return yaw;
}

frc::Pose2d EctoSwerve::getPose() const {
	auto origPose = odometry->GetPose();
	auto pose = frc::Pose2d(origPose.X(), -origPose.Y(), units::angle::radian_t(getYaw()));
	return pose;
}

void EctoSwerve::zeroOdometry() {
	odometry->ResetPosition(frc::Pose2d(units::meter_t(0), units::meter_t(0), frc::Rotation2d(0_deg)),
	                        getPose().Rotation());
}

frc::ChassisSpeeds EctoSwerve::getVelocity() const {
	return currentVelocity;
}


void EctoSwerve::updateOdometry(const SwerveState &motorStates) {
	wpi::array<frc::SwerveModuleState, 4> moduleStates = motorStates.toWPI();
	currentVelocity = kinematics->ToChassisSpeeds(moduleStates);
	
	odometry->Update(frc::Rotation2d(units::angle::radian_t(getYaw())), moduleStates);
}
