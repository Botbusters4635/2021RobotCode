//
// Created by hiram on 13/08/19.
//

#ifdef BOTBUSTERS_REBIRTH_ECTOPWM_H
#include "EctoPWM.h"

EctoPWM::EctoPWM(int id, const std::string &name) : EctoMotor(id, name), PWMSpeedController(id) {
}

void EctoPWM::set(double value) {
	if (!disabled) {
		Set(value * config.motorInverted ? -1.0 : 1.0);
	}
}

void EctoPWM::setClosedLoopOutputRange(double minimum, double maximum) {
	;
}

void EctoPWM::invert(bool state) {
	config.motorInverted = state;
}

bool EctoPWM::isInverted() const {
	return config.motorInverted;
}

bool EctoPWM::isSensorInverted() const {
	return false;
}

void EctoPWM::invertSensor(bool state) {
	;
}

double EctoPWM::getPercentOutput() const {
	return Get();
}

void EctoPWM::setPosition(double position) {
	throw std::logic_error("setPosition is not supported with PWM!");
}

double EctoPWM::getPosition() const {
	throw std::logic_error("getPosition is not supported with PWM!");
}

double EctoPWM::getVelocity() const {
	throw std::logic_error("getVelocity is not supported with PWM!");
}

void EctoPWM::updateControllerConfig() {


}

double EctoPWM::getMotorTemperature() const {
	return 0;
}

double EctoPWM::getMotorCurrent() const {
	return 0;
}

double EctoPWM::getMotorVoltage() const {
	return 0;
}

void EctoPWM::setArbitraryFeedForward(double feedForward) {
	throw std::logic_error("arbitraryFeedForward is not supported with PWM!");
}

void EctoPWM::disable() {
	Set(0);
	disabled = true;
}

void EctoPWM::enableLimitSwitches(bool state) {
	;
}

void EctoPWM::fwdLimitSwitchNormallyOpen(bool state) {
	;
}

void EctoPWM::revLimitSwitchNormallyOpen(bool state) {
	;
}

void EctoPWM::setForwardSoftLimit(double rotations) {
	;
}

void EctoPWM::enableForwardSoftLimit(bool state) {
	;
}

void EctoPWM::setReverseSoftLimit(double rotations) {
	;
}

void EctoPWM::enableReverseSoftLimit(bool state) {
	;
}

void EctoPWM::disableLimits() {
	;
}

void EctoPWM::configureMotionMagicVelocity(double velocity) {
	;
}

void EctoPWM::configureMotionMagicAcceleration(double acceleration) {
	;
}

void EctoPWM::configureMotionMagicSCurve(double sCurve) {
	;
}
#endif