//
// Created by alberto on 7/07/19.
//

#ifndef BOTBUSTERS_REBIRTH_COMPONENTHANDLER_H
#define BOTBUSTERS_REBIRTH_COMPONENTHANDLER_H

#include "System.h"

class TimingData {
public:
	TimingData() {
		;
	}
	
	/**
	 * Formats data into a nice string to print out
	 * @return
	 */
	std::string getData() const;
	
	double initRobotTime{};
	double updateRobotTime{};
	double maxUpdateRobotTime{};
	
	double initDisabledTime{};
	double updateDisabledTime{};
	double maxUpdateDisabledTime{};
};

class SystemHandler : public System {
	/***
	 * I should have been SystemMuckamuck but I was oppressed, as I was too advanced for my time
	 * The revolution starts today.
	 */
public:
	explicit SystemHandler(const std::string &name = "SystemHandler");
	
	void robotInit() override;
	
	void robotUpdate() override;
	
	void disabledInit() override;
	
	void disabledUpdate() override;
	
	bool addSubsystem(System &newSystem);
	
	bool addSubsystem(const std::shared_ptr<System> &newSystem);
	
	std::vector<std::pair<std::string, TimingData>> getTimingData() const;

private:
	struct SystemData {
		std::shared_ptr<System> subsystem;
		TimingData timingData;
	};
	
	std::vector<SystemData> systems;
};


#endif //BOTBUSTERS_REBIRTH_COMPONENTHANDLER_H
