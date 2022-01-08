//
// Created by abiel on 1/2/20.
//

#ifndef BOTBUSTERSREBIRTH_ECTOSWERVE_H
#define BOTBUSTERSREBIRTH_ECTOSWERVE_H

#include <Core/EctoModule/System.h>
#include <Core/EctoInput/Axis/JoystickAxisExpo.h>
#include <Core/EctoInput/Buttons/EctoButton.h>

#include <Control/SimpleControllerSource.h>
#include <Control/SimpleControllerOutput.h>

#include <Control/Kinematics/Swerve/SwerveKinematics.h>
#include <Control/Kinematics/Swerve/SwerveState.h>

#include <Control/Odometry/LinearOdometry.h>
#include <Control/Odometry/ExponentialOdometry.h>

#include "Core/EctoInput/InputManager.h"
#include <Core/MotorHandler/MotorManager.h>
#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DigitalOutput.h>
#include "Sensors/EctoDistanceSensor.h"

#include "Math/EctoMath.h"

#include "Control/EctoPID/PIDConfig.h"

#include <adi/ADIS16470_IMU.h>

#include <frc/smartdashboard/SendableChooser.h>

#include <frc/Notifier.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/geometry/Pose2d.h>

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>

#include "GenericSwerveValue.h"

#include <functional>

#include "Control/Kinematics/Swerve/SwerveKinematics.h"

#include "SwerveModule/SwerveModule.h"
#include "Utilities/NetworkTablePID/NetworkTablePID.h"

struct EctoSwerveConfig {
	//Angles to which the robot will snap to
	std::vector<double> snappableAngles{0, 2.69, M_PI / 2.0, 0.58, M_PI, -0.58, -M_PI / 2.0, -2.72};
	
	double gearRatio = 8.33;
	double wheelCircumference = 0.0508 * M_PI;
	double length = 1.0;
	double width = 1.0;
};

class EctoSwerve : public System {
public:
	explicit EctoSwerve(const EctoSwerveConfig &config);
	
	void robotInit() override;
	
	void robotUpdate() override;
	
	void zeroYaw();
	
	void zeroOdometry();
	
	void setYaw(double yaw);
	
	double getYaw(bool useZero = true) const;
	
	double simYaw{0};
	
	double getYawRate() const {
		double rate = 0;

#ifndef SIMULATION
		rate = (adis->GetRate() / 180.0) * M_PI;
#endif
		
		return rate;
	}
	
	void setVelocity(const frc::ChassisSpeeds &target);
	
	void setPercent(const frc::ChassisSpeeds &target, const Point2D &point = Point2D(0, 0));
	
	void setVoltage(const frc::ChassisSpeeds &target);
	
	void setModules(const SwerveState &setpoint, MotorControlMode controlMode);
	
	void setModules(const std::array<frc::SwerveModuleState, 4> &state, bool velocityControl = true) {
		setModules(SwerveState(state), velocityControl);
	}
	
	void setModules(const SwerveState &rawSetpoint, bool velocityControl = true) {
		setModules(rawSetpoint, velocityControl ? MotorControlMode::Velocity : MotorControlMode::Percent);
	}
	
	frc::Pose2d getPose() const;
	
	SwerveState getMotorStates() const;
	
	frc::ChassisSpeeds getVelocity() const;
	
	std::shared_ptr<frc::SwerveDriveKinematics<4>> getKinematics() const { return kinematics; }

private:
	void initNetworkTables();
	
	void updateNetworkTables();
	
	nt::NetworkTableInstance ntInstance = nt::NetworkTableInstance::GetDefault();
	std::shared_ptr<nt::NetworkTable> table;
	
	EctoSwerveConfig config;
	
	//TODO Implement odometry for setTargetPosition
	/**
	 * Kinematics
	 */
	std::shared_ptr<frc::SwerveDriveKinematics<4>> kinematics;
	
	static std::shared_ptr<frc::SwerveDriveKinematics<4>>
	createKinematics(units::meter_t length, units::meter_t width) {
		frc::Translation2d frontLeft{width, length};
		frc::Translation2d frontRight{width, -length};
		frc::Translation2d backLeft{-width, length};
		frc::Translation2d backRight{-width, -length};
		
		return std::make_shared<frc::SwerveDriveKinematics<4>>(frontLeft, frontRight, backLeft, backRight);
	}
	
	/**
	 * PIDS
	 */
	PIDConfig headingPIDConfig;
	PIDConfig wheelVelocityPID;
	
	std::array<std::unique_ptr<SwerveModule>, 4> modules;
	std::unique_ptr<NetworkTablePID> steerPIDNT, wheelPIDNT;
	
	/**
	 * Motors
	 */
	MotorManager &motorHandler = MotorManager::getInstance();
	
	/**
	 * Heading
	 */
	double headingZero = 0.0;

#ifndef SIMULATION
	std::unique_ptr<frc::ADIS16470_IMU> adis{};
#endif
	
	void updateOdometry(const SwerveState &motorStates);
	
	/**
	 * Odometry
	 */
	std::unique_ptr<frc::SwerveDriveOdometry<4>> odometry;
	frc::ChassisSpeeds currentVelocity;
	
	/**
	 * Kinematics optimization
	 */
	SwerveState lastState, lastSetpoint;
	double lastRunTime = 0;
	
	const bool poseido = false;
};


#endif //BOTBUSTERSREBIRTH_ECTOSWERVE_H
