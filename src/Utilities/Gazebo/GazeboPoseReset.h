//
// Created by abiel on 7/16/20.
//

#ifndef BOTBUSTERSREBIRTH_GAZEBOPOSERESET_H
#define BOTBUSTERSREBIRTH_GAZEBOPOSERESET_H

#include <functional>
#include "Core/EctoModule/System.h"
#include "Core/EctoInput/InputManager.h"
#include "Core/EctoInput/Buttons/EctoButton.h"
#include "Math/DataTypes/RobotPose2D.h"

#include <networktables/NetworkTableInstance.h>

/**
 * Resets current odometry pose from gazebo
 */
class GazeboPoseReset : public System {
public:
	GazeboPoseReset(const std::string &gazeboPoseEntryName, const std::function<void(const RobotPose2D &)> &cb);
	
	void robotUpdate() override;

private:
	nt::NetworkTableInstance ntInstance = nt::NetworkTableInstance::GetDefault();
	std::shared_ptr<nt::NetworkTable> table;
	
	InputManager &input = InputManager::getInstance();
	
	EctoButton resetButton;
	bool lastResetButton{false};
	
	std::function<void(const RobotPose2D &)> cb;
};


#endif //BOTBUSTERSREBIRTH_GAZEBOPOSERESET_H
