//
// Created by Neil Rodriguez Murillo on 9/22/21.
//

#ifndef BOTBUSTERS_REBIRTH_ECTOTANK1_H
#define BOTBUSTERS_REBIRTH_ECTOTANK1_H

#include <Control/EctoPID/PIDConfig.h>
#include <Core/EctoModule/System.h>
#include <Core/MotorHandler/EctoMotor/EctoMotor.h>
#include <networktables/NetworkTableInstance.h>


struct EctoTankConfig {
	double wheelDiameter = 1.0;
	
	double trackWidth = 1;
	
	PIDConfig motorPIDConfig;
	
	std::vector<std::shared_ptr<EctoMotor>> leftMotors, rightMotors;
	
};

class EctoTank : public System {

public:
	EctoTank(const EctoTankConfig &config);
	
	void robotInit() override;
	
	void robotUpdate() override;
	
	void setTargetVelocity(double left, double right);
	
	void setPercentage(double left, double right);

private:
	nt::NetworkTableInstance ntInstance = nt::NetworkTableInstance::GetDefault();
	
	EctoTankConfig tankConfig;
	
	std::mutex targetVelocityMutex;
	
	double previousTime;
};


#endif //BOTBUSTERS_REBIRTH_ECTOTANK1_H
