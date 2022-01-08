//
// Created by alberto on 15/08/19.
//

#ifndef ECTOCONTROL_SWERVEMOTORVALUES_H
#define ECTOCONTROL_SWERVEMOTORVALUES_H

#include <Math/DataTypes/Rotation2D.h>
#include <wpi/array.h>
#include <frc/kinematics/SwerveModuleState.h>
#include "SwerveWheel.h"
#include <iostream>
#include <array>

/**
 * Implemented as a class so helper functions can be added later
 *
 */

class SwerveState {
public:
	SwerveState(const SwerveWheel &topLeft, const SwerveWheel &topRight, const SwerveWheel &backLeft,
	            const SwerveWheel &backRight);
	
	SwerveState(const std::array<frc::SwerveModuleState, 4> &states);
	
	SwerveState();
	
	wpi::array<frc::SwerveModuleState, 4> toWPI() const;
	
	SwerveWheel topLeft;
	SwerveWheel topRight;
	
	SwerveWheel backLeft;
	SwerveWheel backRight;
	
	static SwerveState
	optimizeValues_copy(const SwerveState &currentState, const SwerveState &targetState, const SwerveState &lastState,
	                    const SwerveState &lastSet, double dt);
	
	static SwerveState
	optimizeValues(const SwerveState &currentState, const SwerveState &targetState, const SwerveState &lastState,
	               const SwerveState &lastSet, double dt);
	
	static SwerveWheel
	optimizeWheelValue(const SwerveWheel &currentState, const SwerveWheel &targetState, const SwerveWheel &lastState,
	                   const SwerveWheel &lastSet, double dt);
	
	bool operator==(const SwerveState &rhs);
	
	void roundToEpsilon();
	
	friend std::ostream &operator<<(std::ostream &os, const SwerveState &value);
	
	std::array<SwerveWheel *, 4> wheels = {&topLeft, &topRight, &backLeft, &backRight};

private:
	static double getAngleWeight(const SwerveWheel &currentState,
	                             const SwerveWheel &targetState); //Returns a value from 0 to 1 depending on how far off the angles are
	static double getVelocityWeight(const SwerveWheel &currentState,
	                                const SwerveWheel &targetState); //Returns a value from 0 to 1 depending on the velocity delta
	static double getAngleAccelerationWeight(const SwerveWheel &currentState, const SwerveWheel &targetState,
	                                         const SwerveWheel &lastState,
	                                         double dt); //Depends on the necessary change in velocity
	
	
	static double
	calculateWeight(const SwerveWheel &currentState, const SwerveWheel &targetState, const SwerveWheel &lastState,
	                double dt);
	
	static constexpr double angleWeightValue = 1.0;
	static constexpr double velocityWeightValue = 0.001;
	static constexpr double angleAccelerationWeightValue = 0.1;
};

#endif //ECTOCONTROL_SWERVEMOTORVALUES_H
