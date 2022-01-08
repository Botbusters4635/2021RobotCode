//
// Created by abiel on 12/25/19.
//

#include "Control/Odometry/Odometry.h"

void Odometry::updateOdometry_velocity_dt(const Twist2D &velocity) {
	if (!lastRanTimeSet) {
		//Ignore first value and set last ran time
		lastRanTime = std::chrono::high_resolution_clock::now();
		lastRanTimeSet = true;
		return;
	}
	
	const double dt = std::chrono::high_resolution_clock::duration(
			std::chrono::high_resolution_clock::now() - lastRanTime).count();
	updateOdometry_velocity(velocity, dt);
}