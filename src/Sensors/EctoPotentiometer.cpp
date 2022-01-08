//
// Created by hiram on 9/19/19.
//

#include "EctoPotentiometer.h"

EctoPotentiometer::EctoPotentiometer(int port, double potentiometerRange, double lowerVoltageLimit,
                                     double upperVoltageLimit, double voltageOffset) {
	input = std::make_unique<frc::AnalogInput>(port);
	this->upperVoltageLimit = upperVoltageLimit;
	this->voltageOffset = voltageOffset;
	this->lowerVoltageLimit = lowerVoltageLimit;
	this->potentiometerRange = potentiometerRange;
	
	input->SetOversampleBits(6);
	input->SetAverageBits(4);
	
	lastRecordedPosition = getPosition();
	lastPositionTime = frc::Timer::GetFPGATimestamp();
}

double EctoPotentiometer::PIDGet() {
	return getPosition();
}

double EctoPotentiometer::getVoltage() {
	return input->GetVoltage();
}

/**
 * Gets the current position of the Potentiometer
 * @return
 */
double EctoPotentiometer::getPosition() const {
	lastRecordedPosition =
			(input->GetAverageVoltage() * potentiometerRange / (upperVoltageLimit - lowerVoltageLimit)) - voltageOffset;
	lastPositionTime = frc::Timer::GetFPGATimestamp();
	return lastRecordedPosition;
}

/**
 * gets the velocity at which the potentiometer changed states
 * @return
 */
double EctoPotentiometer::getVelocity() const {
	return (getPosition() - lastRecordedPosition) / (frc::Timer::GetFPGATimestamp() - lastPositionTime);
}
