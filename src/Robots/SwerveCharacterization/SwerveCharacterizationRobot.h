//
// Created by abiel on 11/3/21.
//

#ifndef BOTBUSTERS_REBIRTH_SWERVECHARACTERIZATIONROBOT_H
#define BOTBUSTERS_REBIRTH_SWERVECHARACTERIZATIONROBOT_H

#include "Core/EctoRobot.h"
#include "Core/EctoInput/Axis/JoystickAxisExpo.h"
#include "Systems/EctoSwerve/EctoSwerve.h"

#include <logging/SysIdDrivetrainLogger.h>

class SwerveCharacterizationRobot : public EctoRobot {
public:
	SwerveCharacterizationRobot();
	
	void robotInit() override;
	
	void teleopUpdate() override;
	
	void autoInit() override;
	
	void autoUpdate() override;
	
	void disabledInit() override;

protected:
	std::list<MotorInfo> getMotorConfig() override {
		return {{EctoMotorType::SparkMax, "front_left_wheel",  1},
		        {EctoMotorType::SparkMax, "front_right_wheel", 3},
		        {EctoMotorType::SparkMax, "back_left_wheel",   7},
		        {EctoMotorType::SparkMax, "back_right_wheel",  6},
		
		        {EctoMotorType::SparkMax, "front_left_steer",  2},
		        {EctoMotorType::SparkMax, "front_right_steer", 4},
		        {EctoMotorType::SparkMax, "back_left_steer",   8},
		        {EctoMotorType::SparkMax, "back_right_steer",  5},
		};
	};
	
	std::shared_ptr<EctoSwerve> swerve;
	
	JoystickAxisExpo leftSide{0.2, 0.2}, rightSide{0.2, 0.2};
	
	static std::array<frc::SwerveModuleState, 4> tankSwerve(double left, double right) {
		std::array<frc::SwerveModuleState, 4> state;
		for (int i = 0; i < 4; i += 2) {
			state[i].speed = units::meters_per_second_t(left);
			state[i + 1].speed = units::meters_per_second_t(right);
			state[i].angle = frc::Rotation2d();
			state[i + 1].angle = frc::Rotation2d();
		}
		
		return state;
	}
	
	SysIdDrivetrainLogger logger;
};


#endif //BOTBUSTERS_REBIRTH_SWERVECHARACTERIZATIONROBOT_H
