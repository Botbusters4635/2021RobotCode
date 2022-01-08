//
// Created by abiel on 1/6/20.
//

#ifndef BOTBUSTERSREBIRTH_EXTERNALINTAKE_H
#define BOTBUSTERSREBIRTH_EXTERNALINTAKE_H

#include "Core/PCM/PCMManager.h"
#include "Core/MotorHandler/EctoMotor/EctoMotor.h"
#include <Core/EctoModule/System.h>

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>

struct ExternalIntakeConfig {
	std::shared_ptr<EctoMotor> intakeMotor;
	double motorCurrentLimit = 20;
	double motorOpenLoopRampRate = 1;
	
	std::string pistonName;
	bool invertPiston = false;
	bool invertIntake = false;
};

class ExternalIntake : public System {
public:
	ExternalIntake(const ExternalIntakeConfig &config, std::string externalIntakeName);
	
	void robotInit() override;
	
	void robotUpdate() override;
	
	void liftIntake(bool state);
	
	bool isIntakeDown() const;
	
	void setMotor(double value);

private:
	std::string externalIntakeName;
	
	nt::NetworkTableInstance ntInstance = nt::NetworkTableInstance::GetDefault();
	std::shared_ptr<nt::NetworkTable> table;
	
	void updateNetworkTables();
	
	PCMManager &pcm = PCMManager::getInstance();
	
	ExternalIntakeConfig config;
	
	std::shared_ptr<frc::DoubleSolenoid> piston;
	
	std::shared_ptr<EctoMotor> intakeMotor;
};


#endif //BOTBUSTERSREBIRTH_EXTERNALINTAKE_H
