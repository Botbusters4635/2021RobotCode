//
// Created by Neil Rodriguez Murillo on 9/23/21.
//

#ifndef BOTBUSTERS_REBIRTH_CHAPPIE_H
#define BOTBUSTERS_REBIRTH_CHAPPIE_H

#include <Core/EctoRobot.h>
#include <Systems/EctoTank/EctoTank.h>
#include <Systems/EctoTank/EctoTankInputHandler.h>
#include <Systems/Intake/Intake.h>


class Chappie : public EctoRobot {
	Chappie();
	
	void robotInit() override;
	
	void autoInit() override;
	
	void autoUpdate() override;
	
	void teleopInit() override;
	
	void teleopUpdate() override;


protected:
	std::list<MotorInfo> getMotorConfig() override;
	
	std::list<PistonInfo> getPistonConfig() override;


private:
	//MANAGERS
	MotorManager &manager = MotorManager::getInstance();
	PCMManager &pcm = PCMManager::getInstance();
	
	//SYSTEMS & SYSTEM HANDLERS
	std::shared_ptr<EctoTank> tank;
	std::shared_ptr<EctoTankInputHandler> tankInputHandler;
	
	std::shared_ptr<Intake> intake;
	std::shared_ptr<EctoTankInputHandler> intakeInputHandler;
	
};


#endif //BOTBUSTERS_REBIRTH_CHAPPIE_H
