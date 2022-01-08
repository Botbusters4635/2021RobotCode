//
// Created by hiram on 9/19/19.
// Thank alberto for the name
//

#ifndef BOTBUSTERS_REBIRTH_EctoPotentiometer_H
#define BOTBUSTERS_REBIRTH_EctoPotentiometer_H


#include <frc/PIDSource.h>
#include <frc/AnalogInput.h>
#include <Control/EctoControllerSource.h>
#include <frc/Timer.h>


class EctoPotentiometer : public frc::PIDSource, public EctoControllerSource {
public:
	explicit EctoPotentiometer(int port, double potentiometerRange, double lowerVoltageLimit, double upperVoltageLimit,
	                           double voltageOffset);
	
	double getPosition() const override;
	
	double getVelocity() const override;
	
	double getVoltage();
	
	double PIDGet() override;

private:
	std::unique_ptr<frc::AnalogInput> input;
	double potentiometerRange; //Arbitrary Unit
	double upperVoltageLimit;
	double voltageOffset;
	double lowerVoltageLimit;
	
	mutable double lastRecordedPosition = 0;
	mutable double lastPositionTime = 0;
};


#endif //BOTBUSTERS_REBIRTH_EctoPotentiometer_H
