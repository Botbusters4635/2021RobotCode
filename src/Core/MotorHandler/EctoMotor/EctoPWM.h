//
// Created by hiram on 13/08/19.
//

//Commented out :)
#ifdef BOTBUSTERS_REBIRTH_ECTOPWM_H
#define BOTBUSTERS_REBIRTH_ECTOPWM_H


#include <frc/PWMSpeedController.h>
#include "EctoMotor.h"

class EctoPWM : public EctoMotor, private frc::PWMSpeedController {
public:
	EctoPWM(int id, const std::string &name);

	void set(double value) override;
	
	void setClosedLoopOutputRange(double minimum, double maximum) override;
	
	void invert(bool state) override;
	bool isInverted() const override;
	
	void invertSensor(bool state) override;
	bool isSensorInverted() const override;

	double getMotorTemperature() const override;

	double getMotorCurrent() const override;

	double getMotorVoltage() const override;

	void setArbitraryFeedForward(double feedForward) override;

	double getPercentOutput() const override;

	void setPosition(double position) override;

	double getPosition() const override;

	void disable() override;

	double getVelocity() const override;
	
	void enableLimitSwitches(bool state) override;
	void fwdLimitSwitchNormallyOpen(bool state) override;
	void revLimitSwitchNormallyOpen(bool state) override;
	
	void setForwardSoftLimit(double radians) override;
	void enableForwardSoftLimit(bool state) override;
	
	void setReverseSoftLimit(double radians) override;
	void enableReverseSoftLimit(bool state) override;
	
	void disableLimits() override;
	
	void configureMotionMagicVelocity(double velocity) override;
	void configureMotionMagicAcceleration(double acceleration) override;
	
	void configureMotionMagicSCurve(double sCurve) override;

protected:
	bool disabled = false;

	void updateControllerConfig() override;

};


#endif //BOTBUSTERS_REBIRTH_ECTOPWM_H
