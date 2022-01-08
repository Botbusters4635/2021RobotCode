#ifndef BOTBUSTERSREBIRTH_ECTOINPUT_H
#define BOTBUSTERSREBIRTH_ECTOINPUT_H

#include <Core/EctoInput/Buttons/EctoButton.h>
#include <Core/EctoInput/Axis/JoystickAxis.h>
#include <Core/EctoModule/Manager.h>
#include <frc/DriverStation.h>
#include <frc/Joystick.h>
#include <map>
#include <utility>
#include <string>
#include <mutex>
#include <thread>
#include <chrono>

//TODO Reimplement without EctoSettings


/**
 * This class is in charge of managing and updating all EctoButton and JoystickAxis classes for use within other systems.
 */
class InputManager : public Manager<InputManager> {
	friend class Manager<InputManager>;
	
	struct JoystickData {
		std::shared_ptr<frc::Joystick> joystick;
		std::map<std::string, std::vector<EctoButton *>> registeredButtons;
		std::map<std::string, std::vector<JoystickAxis *>> registeredAxes;
	};

public:
	struct JoystickMap {
		std::map<std::string, int> axesMapping;
		std::map<std::string, int> buttonsMapping;
	};
	
	void registerButton(EctoButton &button, const std::string &buttonName, int joystickId = 0);
	
	void registerAxis(JoystickAxis &axis, const std::string &axisName, int joystickId = 0);
	
	/**
	 * Rumble values should be from 0 to 1
	 */
	void setControllerRumble(double leftRumble, double rightRumble, int joystickId = 0);
	
	int getPOVAngleReading(int joystickId = 0);
	
	void addMapping(JoystickMap mapping, int joystickId = 0);

protected:
	void update() override;

private:
	InputManager();
	
	InputManager &operator=(const InputManager &);
	
	std::map<int, JoystickMap> joystickMappings;
	std::map<int, JoystickData> joysticksData;
	frc::DriverStation &driverStation = frc::DriverStation::GetInstance();
};

#endif
