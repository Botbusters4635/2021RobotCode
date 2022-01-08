//
// Created by abiel on 2/10/20.
//

#include "ShooterCharacterizer.h"
#include <frc/DriverStation.h>
#include <frc/Timer.h>

ShooterCharacterizer::ShooterCharacterizer(const ShooterCharacterizerConfig &config) : System("ShooterCharacterizer") {
	this->config = config;
	config.rightMotor->prioritizeUpdateRate();
	config.leftMotor->prioritizeUpdateRate();
	config.rightMotor->setControlMode(MotorControlMode::Voltage);
	config.leftMotor->setControlMode(MotorControlMode::Voltage);
}

void ShooterCharacterizer::robotInit() {
	std::ofstream propertiesFile;
	propertiesFile.open(config.propertiesFileName, std::fstream::out | std::fstream::trunc);
	
	propertiesFile << fmt::format("{},{},{}\n", "VoltageStep", "MinimumVoltage", "MaximumVoltage");
	propertiesFile << fmt::format("{},{},{}\n", config.voltageStep, config.minimumVoltage, config.maximumVoltage);
	
	propertiesFile.close();
}

double ShooterCharacterizer::getAverageVelocity() const {
	return config.leftMotor->getVelocity();
	//return (config.leftMotor->getVelocity() + config.rightMotor->getVelocity()) / 2.0;
}

double ShooterCharacterizer::getAverageVotage() const {
	return (config.leftMotor->getVoltage() +
	        config.rightMotor->getVoltage()) / 2.0;
}

double ShooterCharacterizer::getAverageCurrent() const {
	return (config.leftMotor->getCurrent() + config.rightMotor->getCurrent()) / 2.0;
}


void ShooterCharacterizer::robotUpdate() {
	if (frc::DriverStation::GetInstance().IsEnabled()) {
		const double averageVelocity = getAverageVelocity();
		const double averageCurrent = getAverageCurrent();
		const double averageVoltage = getAverageVotage();
		
		const double time = frc::Timer::GetFPGATimestamp();
		const double acceleration = (averageVelocity - lastVelocity) / (time - lastTime);
		
		const double dt = frc::Timer::GetFPGATimestamp() - startTime;
		
		switch (currentState) {
			case ShooterCharacterizerState::Initialize:
				
				file.open(config.fileName, std::fstream::out | std::fstream::trunc);
				
				if (!file.is_open()) {
					log->error("File: {} not good", config.fileName);
				}
				
				file << fmt::format("{},{},{},{},{},{}\n", "Time", "Velocity", "SetVoltage", "MeasuredVoltage",
				                    "Current", "Acceleration");
				
				log->info("Initialized!");
				
				startTime = frc::Timer::GetFPGATimestamp();
				currentVoltage = config.minimumVoltage;
				currentState = ShooterCharacterizerState::RunMotor;
				
				break;
			
			case ShooterCharacterizerState::RunMotor:
				config.leftMotor->set(currentVoltage);
				config.rightMotor->set(-currentVoltage);
				
				file << fmt::format("{},{},{},{},{},{}\n", frc::Timer::GetFPGATimestamp(), averageVelocity,
				                    currentVoltage, averageVoltage, averageCurrent, acceleration);
				
				if (dt > config.timeToRunFor) {
					currentState = ShooterCharacterizerState::Deaccelerate;
				}
				
				if (currentState != lastState) {
					log->info("Running at: {} Average Velocity: {}", currentVoltage, averageVelocity);
				}
				
				break;
			
			case ShooterCharacterizerState::Deaccelerate:
				config.leftMotor->set(0);
				config.rightMotor->set(0);
				
				log->info("Deaccelerating");
				
				//Wait for velocity to be about 0 and increase voltage if needed
				if (std::abs(averageVelocity) < 1.0) {
					if (timesRan >= 4) {
						//Velocity = 0
						currentVoltage += config.voltageStep;
						timesRan = 0;
					} else {
						timesRan++;
					}
					
					
					if (currentVoltage > config.maximumVoltage) {
						//Finished
						currentState = ShooterCharacterizerState::Finished;
					} else {
						startTime = frc::Timer::GetFPGATimestamp();
						currentState = ShooterCharacterizerState::RunMotor;
					}
				}
				
				
				break;
			
			case ShooterCharacterizerState::Finished:
				file.close();
				log->info("ShooterCharacterizer finished!");
				
				break;
		}
		
		lastTime = frc::Timer::GetFPGATimestamp();
		lastVelocity = averageVelocity;
		lastState = currentState;
	}
}

