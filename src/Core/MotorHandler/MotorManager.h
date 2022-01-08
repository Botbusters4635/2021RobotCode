//
// Created by alberto on 31/07/19.
//
#ifndef ECTOCONTROL_MOTORHANDLER_H
#define ECTOCONTROL_MOTORHANDLER_H

#include "Core/MotorHandler/EctoMotor/EctoMotor.h"
#include "EctoMotor/DataTypes/EctoMotorType.h"
#include <Core/EctoModule/Manager.h>

struct MotorInfo {
	MotorInfo(EctoMotorType type, const std::string &name, int id) {
		this->type = type;
		this->name = name;
		this->id = id;
	}
	
	EctoMotorType type;
	std::string name;
	int id;
};

class MotorManager : public Manager<MotorManager> {
	friend class Manager<MotorManager>;

public:
	std::shared_ptr<EctoMotor> &getMotor(const std::string &name);
	
	void init();
	
	void update() override;
	
	void setMotorInfo(const std::list<MotorInfo> &motorInfo);

private:
	bool hasInit = false;
	
	const std::string talonVersion = "20.1";
	const std::string sparkMaxVersion = "v1.5.2";
	
	std::list<MotorInfo> motorInfo;
	
	void initializeMotors();
	
	std::vector<std::shared_ptr<EctoMotor>> motorControllers;
	
	MotorManager();
};

#endif //ECTOCONTROL_MOTORHANDLER_H
