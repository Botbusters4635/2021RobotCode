//
// Created by abiel on 8/22/19.
//

#ifndef BOTBUSTERSREBIRTH_ECTOPCM_H
#define BOTBUSTERSREBIRTH_ECTOPCM_H

#include <frc/DoubleSolenoid.h>
#include <Core/EctoModule/Manager.h>
#include <string>
#include <map>

struct PistonInfo {
	PistonInfo() {
		;
	}
	
	PistonInfo(const std::string &name, int id1, int id2) {
		this->name = name;
		this->id1 = id1;
		this->id2 = id2;
	}
	
	std::string name{"NA"};
	int id1{-1}, id2{-1};
};

class PCMManager : public Manager<PCMManager> {
	friend class Manager<PCMManager>;

public:
	std::shared_ptr<frc::DoubleSolenoid> getPiston(const std::string &name);
	
	void init();
	
	void setPCMConfig(const std::list<PistonInfo> &pistonInfo);
	
	//Wrapper for legacy code
	static void set(const std::shared_ptr<frc::DoubleSolenoid> &solenoid, bool state) {
		auto val = state ? frc::DoubleSolenoid::Value::kForward : frc::DoubleSolenoid::Value::kReverse;
		solenoid->Set(val);
	}

protected:
	void update() override;

private:
	PCMManager();
	
	bool hasInit = false;
	
	PCMManager &operator=(const PCMManager &);
	
	void initializePistons();
	
	std::list<PistonInfo> pistonInfo;
	std::map<std::string, std::shared_ptr<frc::DoubleSolenoid>> pistons;
};

#endif //BOTBUSTERSREBIRTH_ECTOPCM_H
