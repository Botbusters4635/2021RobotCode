//
// Created by ajahueym on 1/10/21.
//

#include "EctoGazeboMotor.h"
#include <cmath>
#include <frc/system/plant/DCMotor.h>
#include <frc/DriverStation.h>
#include "Math/EctoMath.h"
#include <algorithm>

int EctoGazeboMotor::ectoGazeboId = 0;
EctoGazeboMotor::UpdateManager &EctoGazeboMotor::updateManager = EctoGazeboMotor::UpdateManager::getInstance();
static const frc::DriverStation &driverStation = frc::DriverStation::GetInstance();

EctoGazeboMotor::EctoGazeboMotor(const std::string &robot, const std::string &motor) : EctoMotor(ectoGazeboId++, motor,
                                                                                                 EctoMotorType::Simulated) {
	auto ntInstance = nt::NetworkTableInstance::GetDefault();
	motorTable = ntInstance.GetTable(robot)->GetSubTable(motor);
	positionEntry = motorTable->GetEntry("Position");
	velocityEntry = motorTable->GetEntry("Velocity");
	voltageEntry = motorTable->GetEntry("Voltage");
	potPositionOffset = 0.0;
	
	ntInstance.AddConnectionListener([this](const nt::ConnectionNotification &notification) {
		if (notification.connected) {
			quadPositionOffset = positionEntry.GetDouble(0.0);
		}
	}, false);
	
	updateManager.addGazeboMotor(this);

#ifndef SIMULATION
	log->warn("Using EctoGazeboMotor in a non simulated project! This is for Gazebo and Simulations! Motor Name: {}",
	          robot + "/" + motor);
#endif
}

void EctoGazeboMotor::factoryReset() {
	motorInverted = false;
	sensorInverted = false;
	encoderCodes = 1;
	quadPositionOffset = positionEntry.GetDouble(0.0);
	potPositionOffset = 0.0;
	voltageEntry.SetDouble(0.0);
	controller = {0.0, 0.0, 0.0, units::millisecond_t(EctoRobotConfig::UPDATE_RATE_MS)};
}

void EctoGazeboMotor::setLimitSwitchPolarity(bool switchPolarity) {
}

std::string EctoGazeboMotor::getFirmwareVersion() const {
	return "1.0.0";
}

void EctoGazeboMotor::setVoltageOutput(double voltage) {
	voltageEntry.SetDouble(voltage);
}

void EctoGazeboMotor::invertMotor(bool state) {
	motorInverted = state;
}

bool EctoGazeboMotor::isMotorInverted() const {
	return motorInverted;
}

void EctoGazeboMotor::invertSensor(bool state) {
	sensorInverted = state;
}

bool EctoGazeboMotor::isSensorInverted() const {
	return sensorInverted;
}

void EctoGazeboMotor::setPIDConfig(const PIDConfig &pidConfig, int profileSlot) {
	std::lock_guard guard(pidMutex);
	
	controller.SetPID(pidConfig.p, pidConfig.i, pidConfig.d);
	if (pidConfig.continous) {
		controller.EnableContinuousInput(pidConfig.minInput, pidConfig.maxInput);
	}
	
	if (pidConfig.maxAbsoluteIntegral != -1.0) {
		if (pidConfig.maxAbsoluteIntegral < 0.0) {
			throw std::invalid_argument(
					"PIDConfig given with negative Max Absolute Integral on Gazebo Motor " + getName());
		}
		controller.SetIntegratorRange(0.0, pidConfig.maxAbsoluteIntegral);
	}
	
	if (pidConfig.clamped) {
		setClosedLoopOutputRange(pidConfig.minOutput, pidConfig.maxOutput);
	}
	
	if (pidConfig.deadband != 0) {
		// TODO Implement
		log->error("Cannot apply deadband on PID output for EctoGazeboMotor {}", getName());
	}
	
}

void EctoGazeboMotor::enableBrakingOnIdle(bool state) {
	log->warn("enableBrakingOnIdle not implemented for EctoGazeboMotor {}", getName());
}

void EctoGazeboMotor::enableCurrentLimit(bool state) {
	log->warn("Current limiting Gazebo Motor... (It does absolutely nothing right now)");
}

void EctoGazeboMotor::setMotorCurrentLimit(double current) {
	log->info("Setting current limit on Gazebo Motor... (It does absolutely nothing right now)");
}

void EctoGazeboMotor::setMotorOutputByCurrent(double amps) {
	std::lock_guard guard(pidMutex);
	controller.SetSetpoint(amps);
}

void EctoGazeboMotor::setClosedLoopOutputRange(double minimum, double maximum) {
	
	if (minimum >= maximum) {
		log->warn(
				"Minimum closed loop output range cant be greater than maximum! Setting not applied on Gazebo Motor {}",
				getName());
	} else {
		minOutput = minimum;
		maxOutput = maximum;
	}
	
}

void EctoGazeboMotor::setClosedLoopRampRate(double rampRate) {
	log->warn("setClosedLoopRampRate not implemented for EctoGazeboMotor {}", getName());
}

void EctoGazeboMotor::setOpenLoopRampRate(double rampRate) {
}

void EctoGazeboMotor::setOutputPercent(double value) {
	setVoltageOutput(value * 12.0);
}

double EctoGazeboMotor::getOutputPercent() const {
	return getVoltage() / 12.0;
}

void EctoGazeboMotor::setSensorPosition(double position) {
	
	switch (feedbackMode) {
		case MotorFeedbackMode::None: {
			log->warn("Setting sensor position when no Feedback Mode was specified for Gazebo Motor {}", getName());
			break;
		}
		case MotorFeedbackMode::QuadEncoder: {
			double rawPosition = positionEntry.GetDouble(0.0);
			quadPositionOffset = rawPosition - position;
			break;
		}
		case MotorFeedbackMode::Potentiometer: {
			double rawPosition = getRawWrappedPotPosition();
			
			if (std::abs(position) > M_PI) {
				throw std::invalid_argument(
						"Magnitude of position cant be greater tha PI using analog sensor on Gazebo Motor " +
						getName());
			}
			
			potPositionOffset = rawPosition - position;
			break;
		}
	}
}

void EctoGazeboMotor::setPositionSetpoint(double position) {
	std::lock_guard guard(pidMutex);
	controller.SetSetpoint(position);
}

void EctoGazeboMotor::setVelocitySetpoint(double velocity) {
	std::lock_guard guard(pidMutex);
	controller.SetSetpoint(velocity);
}

double EctoGazeboMotor::getTemperature() const {
	return 30.0;
}

double EctoGazeboMotor::getCurrent() const {
	// TODO Make it so the motor type can be defined through the Gazebo Plugin
	return frc::DCMotor::NEO().Current(units::radians_per_second_t(getVelocity()),
	                                   units::volt_t(getVoltage())).to<double>();
}

double EctoGazeboMotor::getVoltage() const {
	return voltageEntry.GetDouble(0.0) * (motorInverted ? -1.0 : 1.0);
}

void EctoGazeboMotor::setEncoderCodesPerRev(int codes) {
	encoderCodes = codes;
}

int EctoGazeboMotor::getEncoderCodesPerRev() const {
	return encoderCodes;
}

void EctoGazeboMotor::setArbitraryFeedForward(double feedForward) {
	arbitraryFeedForward = feedForward;
}

void EctoGazeboMotor::disable() {
	setVoltageOutput(0.0);
	disabled = true;
}

bool EctoGazeboMotor::isDisabled() const {
	return disabled;
}

void EctoGazeboMotor::enableLimitSwitches(bool state) {
	log->warn("enableLimitSwitches not implemented for EctoGazeboMotor ", getName());
}

bool EctoGazeboMotor::getForwardLimitSwitch() const {
	log->warn("getForwardLimitSwitch not implemented for EctoGazeboMotor ", getName());
	return false;
}

bool EctoGazeboMotor::getReverseLimitSwitch() const {
	log->warn("getReverseLimitSwitch not implemented for EctoGazeboMotor ", getName());
	return false;
}

void EctoGazeboMotor::setForwardSoftLimit(double radians) {
	log->warn("setForwardSoftLimit not implemented for EctoGazeboMotor ", getName());
}

void EctoGazeboMotor::enableForwardSoftLimit(bool state) {
	log->warn("enableForwardSoftLimit not implemented for EctoGazeboMotor ", getName());
}

void EctoGazeboMotor::setReverseSoftLimit(double radians) {
	log->warn("setReverseSoftLimit not implemented for EctoGazeboMotor ", getName());
}

void EctoGazeboMotor::enableReverseSoftLimit(bool state) {
	log->warn("enableReverseSoftLimit not implemented for EctoGazeboMotor ", getName());
}

void EctoGazeboMotor::configureMotionMagicVelocity(double velocity) {
	log->warn("configureMotionMagicVelocity not implemented for EctoGazeboMotor ", getName());
}

void EctoGazeboMotor::configureMotionMagicAcceleration(double acceleration) {
	log->warn("configureMotionMagicAcceleration not implemented for EctoGazeboMotor ", getName());
}

void EctoGazeboMotor::configureMotionMagicSCurve(double sCurve) {
	log->warn("configureMotionMagicSCurve not implemented for EctoGazeboMotor ", getName());
}

void EctoGazeboMotor::setMotionMagicOutput(double value) {
	log->warn("setMotionMagicOutput not implemented for EctoGazeboMotor ", getName());
}

void EctoGazeboMotor::setAnalogPositionConversionFactor(double conversionFactor) {
	log->warn("setAnalogPositionConversionFactor not implemented for EctoGazeboMotor ", getName());
}

void EctoGazeboMotor::setAnalogVelocityConversionFactor(double conversionFactor) {
	log->warn("setAnalogVelocityConversionFactor not implemented for EctoGazeboMotor ", getName());
}

double EctoGazeboMotor::getRawAnalogPosition() const {
	return getRawWrappedPotPosition();
}

double EctoGazeboMotor::getPotPosition() const {
	double position = (getRawWrappedPotPosition() / encoderCodes - potPositionOffset) * (sensorInverted ? -1.0 : 1.0);
	if (std::abs(position) > M_PI) {
		position -= std::copysign(2 * M_PI, position);
	}
	return position;
}

double EctoGazeboMotor::getPotVelocity() const {
	return (velocityEntry.GetDouble(0.0) / encoderCodes) * (sensorInverted ? -1.0 : 1.0);
}

double EctoGazeboMotor::getQuadPosition() const {
	return (positionEntry.GetDouble(0.0) / encoderCodes - quadPositionOffset) * (sensorInverted ? -1.0 : 1.0);
}

double EctoGazeboMotor::getQuadVelocity() const {
	return (velocityEntry.GetDouble(0.0) / encoderCodes) * (sensorInverted ? -1.0 : 1.0);
}

void EctoGazeboMotor::setPotAsClosedLoopSource() {
	feedbackMode = MotorFeedbackMode::Potentiometer;
}

void EctoGazeboMotor::setQuadAsClosedLoopSource() {
	feedbackMode = MotorFeedbackMode::QuadEncoder;
}

void EctoGazeboMotor::setAnalogSensorOffset(double analogVoltageOffset) {
	
	if (std::abs(analogVoltageOffset) > M_PI) {
		log->warn("Analog Sensor Offset magnitude cant be greater than PI, setting not applied, on Gazebo Motor {}",
		          getName());
	} else {
		potPositionOffset = analogVoltageOffset;
	}
}

void EctoGazeboMotor::followMotor(const EctoMotor &masterMotor, bool isInverted) {
	log->warn("followMotor not implemented for EctoGazeboMotor {}", getName());
}

void EctoGazeboMotor::enableVoltageCompensation(double nominalVoltage) {
	log->info("Enabling voltage compensation on Gazebo Motor... (It does absolutely nothing right now)");
}

void EctoGazeboMotor::prioritizeUpdateRate() {
	log->info("Prioritizing update rate on Gazebo Motor... (It does absolutely nothing right now)");
}

bool EctoGazeboMotor::isInClosedLoop() const {
	return controlMode != MotorControlMode::Percent && controlMode != MotorControlMode::Voltage;;
}

double EctoGazeboMotor::getRawWrappedPotPosition() const {
	double rawPosition = positionEntry.GetDouble(0.0);
	return EctoMath::wrapAngle(rawPosition);
}

EctoGazeboMotor::~EctoGazeboMotor() {
	updateManager.removeGazeboMotor(this);
}


EctoGazeboMotor::UpdateManager::UpdateManager() {
	internalControlUpdateThread = std::thread([this] {
		
		while (runningUpdateThreads) {
			for (const auto &motor: createdGazeboMotors) {
				if (motor->isInClosedLoop() && driverStation.IsEnabled()) {
					std::lock_guard guard(motor->pidMutex);
					
					double voltage = 0.0;
					switch (motor->getControlMode()) {
						
						case MotorControlMode::Velocity:
							voltage = motor->controller.Calculate(motor->getVelocity());
							break;
						case MotorControlMode::Position:
							voltage = motor->controller.Calculate(motor->getPosition());
							break;
						case MotorControlMode::Current:
							voltage = motor->controller.Calculate(motor->getCurrent());
							break;
						case MotorControlMode::MotionMagic:
							throw std::runtime_error(
									"Motion Magic not yet implemented in Gazebo Motor " + motor->getName());
						case MotorControlMode::Percent:
						case MotorControlMode::Voltage:
							throw std::runtime_error(
									"This code is supposed to run in a closed loop control mode! Check EctoGazeboMotor::isInClosedLoop on motor " +
									motor->getName());
					}
					motor->setVoltageOutput(voltage);
				} else if (!driverStation.IsEnabled()) {
					motor->setVoltageOutput(0.0);
				}
			}
			
			std::this_thread::sleep_for(std::chrono::milliseconds((int) EctoRobotConfig::UPDATE_RATE_MS));
		}
	});
}


void EctoGazeboMotor::UpdateManager::addGazeboMotor(EctoGazeboMotor *motor) {
	const auto &iterator = std::find(createdGazeboMotors.begin(), createdGazeboMotors.end(), motor);
	if (iterator != createdGazeboMotors.end()) {
		log->warn("Tried to add a Gazebo Motor twice to the UpdateManager!");
		return;
	}
	createdGazeboMotors.emplace_back(motor);
}

void EctoGazeboMotor::UpdateManager::removeGazeboMotor(EctoGazeboMotor *motor) {
	const auto &iterator = std::find(createdGazeboMotors.begin(), createdGazeboMotors.end(), motor);
	if (iterator != createdGazeboMotors.end()) {
		log->warn("Tried remove a Gazebo Motor that wasn't in UpdateManager!");
		return;
	}
	createdGazeboMotors.erase(iterator);
}

EctoGazeboMotor::UpdateManager::~UpdateManager() {
	runningUpdateThreads = false;
	
	while (!internalControlUpdateThread.joinable()) {
		std::this_thread::sleep_for(std::chrono::milliseconds(50));
	}
	internalControlUpdateThread.join();
}
