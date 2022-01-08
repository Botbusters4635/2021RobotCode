//
// Created by Abiel on 8/22/19.
//
#include "EctoMotorConfig.h"

void EctoMotorConfig::modifyProperty(const std::string &serializedProperty, const std::string &property) {
	if (property == EctoMotorConfigPropertyDict::name) {
		this->name = serializedProperty;
	}
}