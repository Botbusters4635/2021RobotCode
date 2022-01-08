//
// Created by Abiel on 9/9/19.
//

#ifndef BOTBUSTERSREBIRTH_ELEVATOR_H
#define BOTBUSTERSREBIRTH_ELEVATOR_H

#include "Core/EctoModule/System.h"
#include "Core/MotorHandler/EctoMotor/EctoSpark.h"
#include <networktables/NetworkTableEntry.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Timer.h>

class Elevator : public System {
public:
	explicit Elevator(const std::shared_ptr<EctoMotor> &motor);
	
	explicit Elevator() = delete;
	
	void climb(double output);

private:
	std::shared_ptr<EctoMotor> motor;
};


#endif //BOTBUSTERSREBIRTH_ELEVATOR_H
