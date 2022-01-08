//
// Created by hiram on 8/08/19.
//

#ifndef BOTBUSTERS_REBIRTH_ECTOMOTOR_H
#define BOTBUSTERS_REBIRTH_ECTOMOTOR_H

#include <Control/EctoControllerOutput.h>
#include <Control/EctoControllerSource.h>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <rev/CANSparkMax.h>
#include "DataTypes/EctoMotorHealth.h"
#include "DataTypes/EctoMotorMode.h"
#include "DataTypes/EctoMotorConfig.h"
#include "DataTypes/MotorFeedbackMode.h"
#include <frc/PIDOutput.h>
#include <frc/PIDSource.h>

/**
 * Base structure for a motor controller
 */
class EctoMotor :
		public EctoControllerOutput,
		public EctoControllerSource {
public:
	explicit EctoMotor(int motorID, const std::string &motorName, EctoMotorType motorType = EctoMotorType::Empty);
	
	EctoMotorType getMotorType() const;
	
	std::string getName() const;
	
	virtual void factoryReset() = 0;
	
	void setControlMode(MotorControlMode controlMode);
	
	MotorControlMode getControlMode() const;
	
	/**
	 * set the feedback mode used for frc::PIDSource
	 * @param feedbackMode
	 */
	void setFeedbackMode(MotorFeedbackMode feedbackMode);
	
	MotorFeedbackMode getFeedbackMode() const;
	
	/**
	 * True is normally closed, false is normally opened
	 * TODO create enum for this
	 */
	virtual void setLimitSwitchPolarity(bool switchPolarity) = 0;
	
	double getPosition() const override;
	
	double getVelocity() const override;
	
	int getId() const;
	
	void set(double value);
	
	void set(double value, MotorControlMode newControlMode);
	
	virtual std::string getFirmwareVersion() const = 0;
	
	virtual void invertMotor(bool state) = 0;
	
	virtual bool isMotorInverted() const = 0;
	
	virtual void invertSensor(bool state) = 0;
	
	virtual bool isSensorInverted() const = 0;
	
	virtual void setPIDConfig(const PIDConfig &pidConfig, int profileSlot = 0) = 0;
	
	virtual void enableBrakingOnIdle(bool state) = 0;
	
	virtual void enableCurrentLimit(bool state) = 0;
	
	virtual void setMotorCurrentLimit(double amps) = 0;
	
	virtual void setClosedLoopOutputRange(double minimum, double maximum) = 0;
	
	virtual void setClosedLoopRampRate(double rampRate) = 0;
	
	virtual void setOpenLoopRampRate(double rampRate) = 0;
	
	/**
	 * set the current position in radians
	 * @param position
	 */
	virtual void setSensorPosition(double position) = 0;
	
	virtual double getTemperature() const = 0;
	
	virtual double getCurrent() const = 0;
	
	virtual double getVoltage() const = 0;
	
	virtual double getOutputPercent() const = 0;
	
	virtual void setEncoderCodesPerRev(int codes) = 0;
	
	virtual int getEncoderCodesPerRev() const = 0;
	
	/**
	 * (In volts)
	 * Feedforward is not set until the next set command is sent
	 */
	virtual void setArbitraryFeedForward(double feedForward) = 0;
	
	virtual void disable() = 0;
	
	virtual bool isDisabled() const = 0;
	
	virtual void enableLimitSwitches(bool state) = 0;
	
	virtual bool getForwardLimitSwitch() const = 0;
	
	virtual bool getReverseLimitSwitch() const = 0;
	
	virtual void setForwardSoftLimit(double radians) = 0;
	
	virtual void enableForwardSoftLimit(bool state) = 0;
	
	virtual void setReverseSoftLimit(double radians) = 0;
	
	virtual void enableReverseSoftLimit(bool state) = 0;
	
	virtual void configureMotionMagicVelocity(double velocity) = 0;
	
	virtual void configureMotionMagicAcceleration(double acceleration) = 0;
	
	virtual void configureMotionMagicSCurve(double sCurve) = 0;
	
	virtual void setAnalogPositionConversionFactor(double conversionFactor) = 0;
	
	virtual void setAnalogVelocityConversionFactor(double conversionFactor) = 0;
	
	virtual void setAnalogSensorOffset(double analogVoltageOffset) = 0;
	
	virtual void followMotor(const EctoMotor &masterMotor, bool isInverted) = 0;
	
	virtual void enableVoltageCompensation(double nominalVoltage) = 0;
	
	/**
	 * Increases the update rate at which motor velocity is sent back to the RoboRio
	 * Might overload the CAN bus if overused
	 */
	//NOTE: Very experimental and probably should get a better implementation
	virtual void prioritizeUpdateRate() = 0;
	
	std::pair<MotorControlMode, double> getLastSetpoint() const;
	
	void outputSet(double set) override;

protected:
	
	virtual double getRawAnalogPosition() const = 0;
	
	virtual double getPotPosition() const = 0;
	
	virtual double getPotVelocity() const = 0;
	
	virtual double getQuadPosition() const = 0;
	
	virtual double getQuadVelocity() const = 0;
	
	virtual void setVoltageOutput(double voltage) = 0;
	
	virtual void setMotorOutputByCurrent(double amps) = 0;
	
	virtual void setOutputPercent(double value) = 0;
	
	virtual void setPositionSetpoint(double position) = 0;
	
	virtual void setVelocitySetpoint(double velocity) = 0;
	
	virtual void setMotionMagicOutput(double value) = 0;
	
	virtual void setPotAsClosedLoopSource() = 0;
	
	virtual void setQuadAsClosedLoopSource() = 0;
	
	int id;
	EctoMotorType motorType;
	std::string name;
	MotorControlMode controlMode = MotorControlMode::Percent;
	MotorFeedbackMode feedbackMode = MotorFeedbackMode::QuadEncoder;
	std::pair<MotorControlMode, double> lastSetpoint{};
	static std::shared_ptr<spdlog::logger> log;
	bool disabled = false;
};

#endif //BOTBUSTERS_REBIRTH_ECTOMOTOR_H
