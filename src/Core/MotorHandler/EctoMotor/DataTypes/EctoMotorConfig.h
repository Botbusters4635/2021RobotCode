//
// Created by alberto on 13/07/19.
//

#ifndef MOTORTRANSLATOR_ECTOMOTORCONFIG_H
#define MOTORTRANSLATOR_ECTOMOTORCONFIG_H

#include "Control/EctoPID/EctoPID.h"
#include "EctoMotorMode.h"
#include "EctoMotorType.h"

struct EctoMotorConfig {
	std::string name = "";
	EctoMotorType type;
	int id = -1;
	
	void modifyProperty(const std::string &serializedProperty, const std::string &property);
};

namespace EctoMotorConfigPropertyDict {
	const std::string name = "name";
	const std::string id = "id";
}

#endif //MOTORTRANSLATOR_ECTOMOTORCONFIG_H
