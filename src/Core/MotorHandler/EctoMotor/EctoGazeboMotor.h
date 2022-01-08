//
// Created by ajahueym on 1/10/21.
//

#ifndef BOTBUSTERS_REBIRTH_ECTOGAZEBOMOTOR_H
#define BOTBUSTERS_REBIRTH_ECTOGAZEBOMOTOR_H

#include "EctoMotor.h"
#include <ntcore.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <frc/controller/PIDController.h>
#include "Core/EctoRobotConfig.h"

class EctoGazeboMotor : public EctoMotor {
public:
	class UpdateManager {
	public:
		void addGazeboMotor(EctoGazeboMotor *motor);
		
		void removeGazeboMotor(EctoGazeboMotor *motor);
		
		static UpdateManager &getInstance() {
			static UpdateManager instance;
			return instance;
		}
		
		UpdateManager(UpdateManager const &) = delete;
		
		UpdateManager &operator=(UpdateManager const &) = delete;
		
		~UpdateManager();
	
	private:
		explicit UpdateManager();
		
		std::thread internalControlUpdateThread;
		std::atomic<bool> runningUpdateThreads{true};
		std::vector<EctoGazeboMotor *> createdGazeboMotors;
	};
	
	EctoGazeboMotor(const std::string &robot, const std::string &motor);
	
	void factoryReset() override;
	
	void setLimitSwitchPolarity(bool switchPolarity) override;
	
	std::string getFirmwareVersion() const override;
	
	void setVoltageOutput(double voltage) override;
	
	void invertMotor(bool state) override;
	
	bool isMotorInverted() const override;
	
	void invertSensor(bool state) override;
	
	bool isSensorInverted() const override;
	
	void setPIDConfig(const PIDConfig &pidConfig, int profileSlot) override;
	
	void enableBrakingOnIdle(bool state) override;
	
	void enableCurrentLimit(bool state) override;
	
	void setMotorCurrentLimit(double current) override;
	
	void setMotorOutputByCurrent(double value) override;
	
	void setClosedLoopOutputRange(double minimum, double maximum) override;
	
	void setClosedLoopRampRate(double rampRate) override;
	
	void setOpenLoopRampRate(double rampRate) override;
	
	void setOutputPercent(double value) override;
	
	double getOutputPercent() const override;
	
	void setSensorPosition(double position) override;
	
	void setPositionSetpoint(double position) override;
	
	void setVelocitySetpoint(double velocity) override;
	
	double getTemperature() const override;
	
	double getCurrent() const override;
	
	double getVoltage() const override;
	
	void setEncoderCodesPerRev(int codes) override;
	
	int getEncoderCodesPerRev() const override;
	
	void setArbitraryFeedForward(double feedForward) override;
	
	void disable() override;
	
	bool isDisabled() const override;
	
	void enableLimitSwitches(bool state) override;
	
	bool getForwardLimitSwitch() const override;
	
	bool getReverseLimitSwitch() const override;
	
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
	
	double getRawAnalogPosition() const override;
	
	double getPotPosition() const override;
	
	double getPotVelocity() const override;
	
	double getQuadPosition() const override;
	
	double getQuadVelocity() const override;
	
	void setPotAsClosedLoopSource() override;
	
	void setQuadAsClosedLoopSource() override;
	
	void setAnalogSensorOffset(double analogVoltageOffset) override;
	
	void followMotor(const EctoMotor &masterMotor, bool isInverted) override;
	
	void enableVoltageCompensation(double nominalVoltage) override;
	
	void prioritizeUpdateRate() override;
	
	~EctoGazeboMotor();

private:
	
	bool isInClosedLoop() const;
	
	double getRawWrappedPotPosition() const;
	
	static int ectoGazeboId;
	static UpdateManager &updateManager;
	std::shared_ptr<nt::NetworkTable> motorTable;
	nt::NetworkTableEntry positionEntry;
	nt::NetworkTableEntry velocityEntry;
	nt::NetworkTableEntry voltageEntry;
	std::mutex pidMutex;
	bool motorInverted = false;
	bool sensorInverted = false;
	bool disabled = false;
	frc2::PIDController controller{0.0, 0.0, 0.0, units::millisecond_t(EctoRobotConfig::UPDATE_RATE_MS)};
	double minOutput = 1.0, maxOutput = -1.0;
	double arbitraryFeedForward = 0.0;
	int encoderCodes = 1.0;
	double quadPositionOffset = 0.0;
	double potPositionOffset = 0.0;
	
};


#endif //BOTBUSTERS_REBIRTH_ECTOGAZEBOMOTOR_H
