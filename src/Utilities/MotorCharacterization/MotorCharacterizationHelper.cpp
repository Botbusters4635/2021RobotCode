//
// Created by abiel on 9/10/19.
//

#include "MotorCharacterizationHelper.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include <iostream>
#include <fstream>

MotorCharacterizationHelper::MotorCharacterizationHelper(const std::string &name, const std::string &fileName,
                                                         const std::shared_ptr<EctoMotor> &motor) : System(name),
                                                                                                    file(fileName,
                                                                                                         std::fstream::out |
                                                                                                         std::fstream::trunc) {
	this->motor = motor;
	this->fileName = fileName;
	motor->setControlMode(MotorControlMode::Percent);
	
	motor->enableCurrentLimit(true);
	motor->setMotorCurrentLimit(40);
	motor->setOpenLoopRampRate(0.001);
	motor->enableVoltageCompensation(12.0);
}

void MotorCharacterizationHelper::robotInit() {
	log->warn("MotorCharacterizationHelper enabled for motor: {}", motor->getName());
	log->info("Writing to file: {}", fileName);
	//nt::NetworkTableInstance::GetDefault().SetUpdateRate(0.010);
	
	if (!file.is_open()) {
		log->error("File: {} not good", fileName);
	}
	
	file << fmt::format("{},{},{},{},{}\n", "Time", "Position", "Velocity", "Acceleration", "Voltage");
	
	motor->set(0);
	autoSpeedEntry.SetDouble(1);
}

void MotorCharacterizationHelper::robotUpdate() {
	/**
	 * Check if finished
	 */
	if (timesRan >= timesToRun) {
		log->info("Finished running: {}", getName());
		file.close();
		return;
	}
	
	/**
	 * Init Timer
	 */
	if (startTime == 0) {
		startTime = frc::Timer::GetFPGATimestamp();
	}
	
	/**
	 * Wait for the motor to slow down
	 */
	if (waitingToSlowDown) {
		if (waitingToSlowDown and frc::Timer::GetFPGATimestamp() - startTime > timeToWaitAfterRuns) {
			//Finished waiting
			waitingToSlowDown = false;
			startTime = frc::Timer::GetFPGATimestamp();
			lastRunTime = frc::Timer::GetFPGATimestamp();
		} else {
			motor->set(0);
			return;
		}
	} else {
		/**
	 * Check if ran for enough time
	 */
		if (frc::Timer::GetFPGATimestamp() - startTime > timeToRunFor) {
			motor->set(0);
			timesRan++;
			waitingToSlowDown = true;
			startTime = frc::Timer::GetFPGATimestamp();
			log->info("Ran: {} times", timesRan);
			//file.close();
			return;
		}
	}
	
	
	//frc::SmartDashboard::PutNumber("MotorCharacterizationHelper/encoder_position", (motor->getQuadPosition() / (M_PI * 2.0)));
	//frc::SmartDashboard::PutNumber("MotorCharacterizationHelper/encoder_velocity", (motor->getVelocity() / (M_PI * 2.0)));
	//frc::SmartDashboard::PutNumber("MotorCharacterizationHelper/motorVoltage", (motor->getVoltage()));
	
	double autoSpeed = (9.0 / 12.0);
	motor->set(autoSpeed);
	
	const double time = frc::Timer::GetFPGATimestamp();
	const double position = 0;
	const double velocity = motor->getVelocity() / (M_PI * 2.0);
	
	const double acceleration = (velocity - lastVelocity) / (frc::Timer::GetFPGATimestamp() - lastRunTime);
	
	const double voltage = motor->getVoltage();
	
	if (velocity == lastVelocity) {
		return;
	}
	
	lastVelocity = velocity;
	
	file << fmt::format("{},{},{},{},{}\n", time, position, velocity, acceleration, voltage);
	
	lastRunTime = frc::Timer::GetFPGATimestamp();
	
	//double batteryVoltage = frc::RobotController::GetInputVoltage();
	//double motorVoltage = motor->getVoltage();
	
	//log->info("{},{},{},{},{}\n",  time, position, velocity, acceleration, voltage);

//	double numberArray[]{frc::Timer::GetFPGATimestamp(),
//	                     batteryVoltage,
//	                     autoSpeed,
//	                     motorVoltage,
//	                     motor->getPosition() / (M_PI * 2.0),
//	                     motor->getVelocity() / (M_PI * 2.0)};
	
	//telemetryEntry.SetDoubleArray(numberArray);
}