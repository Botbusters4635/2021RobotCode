//
// Created by alberto on 13/07/19.
//

#ifndef MOTORTRANSLATOR_ECTOPID_H
#define MOTORTRANSLATOR_ECTOPID_H

#include <chrono>
#include "Control/EctoController.h"
#include "Control/EctoControllerOutput.h"
#include "Control/EctoControllerSource.h"
#include "PIDConfig.h"
#include <stdexcept>
#include <mutex>

class EctoPID : public EctoController {
public:
	EctoPID(EctoControllerSource &source, EctoControllerOutput &output, const PIDConfig &config);
	
	void update() override;
	
	void setSetpoint(double setpoint) override;
	
	double getSetpoint() const override;
	
	double getError() const override;
	
	double getRawError() const;
	
	double getErrorIntegral() const;
	
	double getErrorDelta() const;
	
	double getMaxError() const;
	
	const PIDConfig &getConfig() const;
	
	void setConfig(const PIDConfig &config);
	
	const EctoControllerOutput &getControllerOutput() const override;
	
	const EctoControllerSource &getControllerSource() const override;

private:
	double setpoint = 0.0f;
	double error = 0.0f;
	double rawError = 0.0f;
	double errorIntegral = 0.0f;
	double errorHai = 0.0f;
	double errorDelta = 0.0f;
	double lastError = 0.0f;
	double maxError = 0.0f;
	
	EctoControllerSource &source;
	EctoControllerOutput &output;
	
	PIDConfig config;
	
	std::chrono::high_resolution_clock::time_point lastTimeUpdate;
	
	mutable std::mutex setPointMutex;
	mutable std::mutex configMutex;
	bool initialTimeSet = false;
};


#endif //MOTORTRANSLATOR_ECTOPID_H
