//
// Created by Neil Rodriguez Murillo on 10/8/21.
//

#ifndef BOTBUSTERS_REBIRTH_INTAKE_H
#define BOTBUSTERS_REBIRTH_INTAKE_H

#include <Core/EctoModule/System.h>
#include <Core/PCM/PCMManager.h>
#include <Core/MotorHandler/EctoMotor/EctoMotor.h>

struct IntakeConfig {
	std::shared_ptr<frc::DoubleSolenoid> piston;
	std::shared_ptr<EctoMotor> motor;
	
	bool invertMotor = false;
	
	double rampRate = 0.18;
	double currentLimit = 10;
};

class Intake : public System {
public:
	Intake(const IntakeConfig &config);
	
	void set(double pct, bool extended);

private:
	//MANAGERS
	PCMManager &pcm = PCMManager::getInstance();
	
	//CONFIGS
	IntakeConfig intakeConfig;
	
	//INSTANCES
//	nt::NetworkTableInstance ntInstance = nt::NetworkTableInstance::GetDefault();
};


#endif //BOTBUSTERS_REBIRTH_INTAKE_H
