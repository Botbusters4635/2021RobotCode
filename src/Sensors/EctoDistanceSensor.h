//
// Created by abiel on 9/23/19.
//

#ifndef BOTBUSTERSREBIRTH_ECTODISTANCESENSOR_H
#define BOTBUSTERSREBIRTH_ECTODISTANCESENSOR_H

#include <frc/AnalogInput.h>
#include <Control/EctoControllerSource.h>

class EctoDistanceSensor : public EctoControllerSource {
public:
	EctoDistanceSensor(double leftZeroOffset, double rightZeroOffset, double w, double maxDistance, double minVoltage,
	                   double maxVoltage);
	
	double getAverageDistance() const;
	
	double getAngle() const;
	
	//Returns angle offset
	double getPosition() const override;
	
	double getVelocity() const override;

private:
	double getLeftSensorDistance() const;
	
	double getRightSensorDistance() const;
	
	double maxVoltage, minVoltage;
	
	double w;
	double leftZeroOffset, rightZeroOffset;
	
	double maxDistance;
	
	const int leftChannel = 1;
	const int rightChannel = 3;
	
	std::unique_ptr<frc::AnalogInput> leftInput, rightInput;
};


#endif //BOTBUSTERSREBIRTH_ECTODISTANCESENSOR_H
