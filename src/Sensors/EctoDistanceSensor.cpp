//
// Created by abiel on 9/23/19.
//

#include "EctoDistanceSensor.h"

EctoDistanceSensor::EctoDistanceSensor(double leftZeroOffset, double rightZeroOffset, double w, double maxDistance,
                                       double minVoltage, double maxVoltage) {
	this->leftZeroOffset = leftZeroOffset;
	this->rightZeroOffset = rightZeroOffset;
	this->w = w;
	
	this->maxDistance = maxDistance;
	this->maxVoltage = maxVoltage;
	this->minVoltage = minVoltage;
	
	leftInput = std::make_unique<frc::AnalogInput>(leftChannel);
	rightInput = std::make_unique<frc::AnalogInput>(rightChannel);
	
	leftInput->SetOversampleBits(4);
	leftInput->SetAverageBits(2);
	
	rightInput->SetOversampleBits(4);
	rightInput->SetAverageBits(2);
}

double EctoDistanceSensor::getAverageDistance() const {
	return (getLeftSensorDistance() + getRightSensorDistance()) / 2.0;
}

double EctoDistanceSensor::getAngle() const {
	return std::atan((getRightSensorDistance() - getLeftSensorDistance()) / w);
}

double EctoDistanceSensor::getPosition() const {
	return getAngle();
}

double EctoDistanceSensor::getVelocity() const {
	//Too lazy to implement this
	throw std::runtime_error("Not implemented");
}

double EctoDistanceSensor::getLeftSensorDistance() const {
	return (leftInput->GetAverageVoltage() * maxDistance / (maxVoltage - minVoltage)) - leftZeroOffset;
}

double EctoDistanceSensor::getRightSensorDistance() const {
	return (rightInput->GetAverageVoltage() * maxDistance / (maxVoltage - minVoltage)) - rightZeroOffset;
}