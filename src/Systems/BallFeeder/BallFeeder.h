//
// Created by abiel on 1/6/20.
//

#ifndef BOTBUSTERSREBIRTH_BALLFEEDER_H
#define BOTBUSTERSREBIRTH_BALLFEEDER_H

#include "Core/MotorHandler/EctoMotor/EctoMotor.h"
#include "Core/EctoModule/System.h"

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>

#include <frc/DigitalInput.h>
#include <frc/DigitalGlitchFilter.h>
#include <frc/Counter.h>

#include <atomic>

struct BallFeederConfig {
	std::shared_ptr<EctoMotor> motor;
	
	double feederCurrentLimit = 55;
	double feederOpenLoopRampRate = 0.04;
};

class BallFeeder : public System {
public:
	BallFeeder(const BallFeederConfig &config);
	
	void feedBalls(double feedSpeed);
	
	void stopBallAtSwitch(bool state);
	
	void stopFeeding();
	
	bool isBallPreloaded() const;
	
	int getBallCount();
	
	void resetBallCount();
	
	void robotInit() override;
	
	void robotUpdate() override;

private:
	nt::NetworkTableInstance ntInstance = nt::NetworkTableInstance::GetDefault();
	std::shared_ptr<nt::NetworkTable> table;
	
	void updateNetworkTables();
	
	std::shared_ptr<EctoMotor> motor;
	
	BallFeederConfig config;
	
	std::atomic<bool> stopAtSwitch{false};
	
	std::shared_ptr<frc::DigitalInput> ballSwitch;
	frc::DigitalGlitchFilter glitchFilter;
	std::unique_ptr<frc::Counter> ballCounter;
};

#endif //BOTBUSTERSREBIRTH_BALLFEEDER_H
