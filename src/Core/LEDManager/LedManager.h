//
// Created by abiel on 2/6/20.
//

#ifndef BOTBUSTERSREBIRTH_LEDMANAGER_H
#define BOTBUSTERSREBIRTH_LEDMANAGER_H

#include <queue>

#include <Core/EctoModule/Manager.h>
#include <string>
#include <mutex>
#include <map>

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>

enum class LEDPattern {
	GreenFire = -1,
	BlueFire = -2,
	RedFire = -3,
	RandomBlink = 2,
	BreatheRandom = 1,
	ShootingDot = 3
};

enum class PatternPriority {
	HighPriority = 0,
	MedPriority = 1,
	LowPriority = 2
};

struct RGBColor {
	RGBColor(double r, double g, double b) {
		this->r = r;
		this->g = g;
		this->b = b;
	}
	
	RGBColor() : RGBColor(0, 0, 0) { ; }
	
	double r, g, b;
};

struct PatternCommand {
	PatternCommand() { ; }
	
	RGBColor primaryColor, secondaryColor;
	LEDPattern primaryPattern, secondaryPattern;
	
	double patternRate{1.0};
};

/**
 * This class manages and sends commands to NetworkTables to be used by an external controller of a LED lightning system.
 */
class LEDManager : public Manager<LEDManager> {
	friend class Manager<LEDManager>;

public:
	void queueCommand(const PatternCommand &command, PatternPriority priority = PatternPriority::MedPriority);
	
	void stopCommandsByPriority(PatternPriority priority);

protected:
	void update() override;
	
	void sendCommand(const PatternCommand &command);

private:
	LEDManager();
	
	std::map<PatternPriority, PatternCommand> commandQueue;
	
	nt::NetworkTableInstance ntInstance = nt::NetworkTableInstance::GetDefault();
	std::shared_ptr<nt::NetworkTable> table;
	
	const std::string baseTableKey = "LedsManager";
	
	const std::string firstKey = "First";
	const std::string secondKey = "Second";
	
	const std::string redKey = "R";
	const std::string greenKey = "G";
	const std::string blueKey = "B";
	
	const std::string isFireEffectKey = "isFired";
	const std::string firePalette = "FirePalette";
	
	const std::string effectKey = "Effect";
	const std::string breatheRateKey = "BreatheRate";
};


#endif //BOTBUSTERSREBIRTH_LEDMANAGER_H
