//
// Created by alberto on 02/09/19.
//

#ifndef SLIDING_MODE_CONTROL_SLIDINGMODECONTROL_H
#define SLIDING_MODE_CONTROL_SLIDINGMODECONTROL_H

#include "Control/EctoController.h"
#include "Control/EctoControllerOutput.h"
#include "Control/EctoControllerSource.h"

struct EctoSMCConfig {
	double k2 = 0.0; //gain PD
	double k3 = 0.0; //Rate of change of adaptive gain'
	double lambda = 0.0; //P
	double kMin = 0.01; //Gain deadband
	double mu = 0.0; //Offset
	
	bool continous = false;
	
	double maxInput = M_PI;
	double minInput = -M_PI;
};

class EctoSMC : public EctoController {
public:
	explicit EctoSMC(EctoControllerSource &source, EctoControllerOutput &output, const EctoSMCConfig &config);
	
	void setSetpoint(double setpoint) override;
	
	double getSetpoint() const override;
	
	double getError() const override;
	
	void update() override;
	
	const EctoControllerOutput &getControllerOutput() const;
	
	const EctoControllerSource &getControllerSource() const;
	
	double getSlidingSurface();
	
	void setConfig(const EctoSMCConfig &config);

private:
	//TODO Maybe move this to EctoMath
	template<typename T>
	int sign(T val) {
		return (T(0) < val) - (val < T(0));
	}
	
	EctoSMCConfig config;
	
	EctoControllerSource &source;
	EctoControllerOutput &output;
	
	double setpoint = 0;
	
	double maxError;
	
	double error = 0;
	double lastError = 0;
	double ka = 0; //Adaptive gain
};


#endif //SLIDING_MODE_CONTROL_SLIDINGMODECONTROL_H
