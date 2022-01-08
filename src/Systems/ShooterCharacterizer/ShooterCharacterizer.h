//
// Created by abiel on 2/10/20.
//

#ifndef BOTBUSTERSREBIRTH_SHOOTERCHARACTERIZER_H
#define BOTBUSTERSREBIRTH_SHOOTERCHARACTERIZER_H

#include "Core/MotorHandler/EctoMotor/EctoMotor.h"
#include <Core/EctoModule/System.h>

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>

#include <iostream>
#include <fstream>

struct ShooterCharacterizerConfig {
	std::string fileName = "/home/lvuser/ShooterCharacterization.csv";
	std::string propertiesFileName = "/home/lvuser/ShooterCharacterizationProperties.csv";
	
	std::shared_ptr<EctoMotor> leftMotor;
	std::shared_ptr<EctoMotor> rightMotor;
	
	double voltageStep = 0.5;
	double timeToRunFor = 10.0;
	
	double minimumVoltage = 1.0;
	double maximumVoltage = 9.5;
};

enum class ShooterCharacterizerState {
	Initialize,
	RunMotor,
	Deaccelerate,
	Finished
};

class ShooterCharacterizer : public System {
public:
	ShooterCharacterizer(const ShooterCharacterizerConfig &config);
	
	void robotInit() override;
	
	void robotUpdate() override;

private:
	ShooterCharacterizerConfig config;
	
	ShooterCharacterizerState currentState = ShooterCharacterizerState::Initialize;
	ShooterCharacterizerState lastState = ShooterCharacterizerState::Initialize;
	
	std::ofstream file;
	
	double getAverageVelocity() const;
	
	double getAverageCurrent() const;
	
	double getAverageVotage() const;
	
	double startTime;
	
	double currentVoltage;
	
	double lastTime{0.001};
	double lastVelocity{0.01};
	
	int timesRan{0};
};


#endif //BOTBUSTERSREBIRTH_SHOOTERCHARACTERIZER_H
