//
// Created by hiram on 8/08/19.
//

#ifndef ECTOCONTROL_ECTOTALON_H
#define ECTOCONTROL_ECTOTALON_H

#include <ctre/Phoenix.h>
#include "EctoMotor.h"

class EctoTalon : public EctoMotor {
public:
	explicit EctoTalon(int id, const std::string &name = " ");
	
	void setClosedLoopOutputRange(double minimum, double maximum) override;
	
	void invertMotor(bool state) override;
	
	bool isMotorInverted() const override;
	
	void invertSensor(bool state) override;
	
	bool isSensorInverted() const override;
	
	void setPIDConfig(const PIDConfig &pidConfig, int profileSlot) override;
	
	void enableBrakingOnIdle(bool state) override;
	
	void enableCurrentLimit(bool state) override;
	
	void setMotorCurrentLimit(double current) override;
	
	void setClosedLoopRampRate(double rampRate) override;
	
	void setOpenLoopRampRate(double rampRate) override;
	
	double getOutputPercent() const override;
	
	void setOutputPercent(double value) override;
	
	void setSensorPosition(double position) override;
	
	void setPositionSetpoint(double position) override;
	
	void setVoltageOutput(double voltage) override;
	
	void setVelocitySetpoint(double value) override;
	
	double getTemperature() const override;
	
	double getCurrent() const override;
	
	void setMotorOutputByCurrent(double value) override;
	
	double getVoltage() const override;
	
	void setArbitraryFeedForward(double feedForward) override;
	
	void disable() override;
	
	void enableLimitSwitches(bool state) override;
	
	void setForwardSoftLimit(double radians) override;
	
	void enableForwardSoftLimit(bool state) override;
	
	void setReverseSoftLimit(double radians) override;
	
	void enableReverseSoftLimit(bool state) override;
	
	void configureMotionMagicVelocity(double velocity) override;
	
	void configureMotionMagicAcceleration(double acceleration) override;
	
	void configureMotionMagicSCurve(double sCurve) override;
	
	void setMotionMagicOutput(double value) override;
	
	void setAnalogPositionConversionFactor(double conversionFactor) override;
	
	void setAnalogVelocityConversionFactor(double conversionFactor) override;
	
	void setEncoderCodesPerRev(int codes) override;
	
	int getEncoderCodesPerRev() const override;
	
	bool isDisabled() const override;
	
	double getPotPosition() const override;
	
	double getPotVelocity() const override;
	
	double getQuadPosition() const override;
	
	double getQuadVelocity() const override;
	
	double getRawAnalogPosition() const override;
	
	void setPotAsClosedLoopSource() override;
	
	void setQuadAsClosedLoopSource() override;
	
	void factoryReset() override;
	
	std::string getFirmwareVersion() const override;
	
	void setLimitSwitchPolarity(bool normallyClosed) override;
	
	bool getReverseLimitSwitch() const override;
	
	bool getForwardLimitSwitch() const override;
	
	void setAnalogSensorOffset(double analogOffset) override;
	
	void followMotor(const EctoMotor &masterMotor, bool inverted = false) override;
	
	void enableVoltageCompensation(double nominalVoltage = 12.0) override;
	
	void prioritizeUpdateRate() override;

protected:
	std::unique_ptr<TalonSRX> talonBase;
	
	double currentFeedForward;
	int encoderCodes = 0;
	bool areLimitSwitchesEnabled = false;
	bool areSoftLimitsEnabled = false;
	bool sensorInverted = false;
};


#endif //ECTOCONTROL_ECTOTALON_H
