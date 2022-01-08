#include <frc/RobotBase.h>
#include "Robots/SimRobot/SimulationRobot.h"
#include <networktables/NetworkTableInstance.h>
#include "Robots/StayPufft.h"
#include "Robots/SwerveCharacterization/SwerveCharacterizationRobot.h"

/**
 * Botbusters Rebirth 2021
 *
 * Created by:
 * Gustavo Ramos Ramos
 * Andres Alarcon Navarro
 * Abiel Fernandez
 * Hiram Muñoz
 * Alberto Jahuey
 * Karen Rodriguez
 *
 *                     __---__
 *                  _-       _--______
 *             __--( /     \ )XXXXXXXXXXXXX_
 *           --XXX(   O   O  )XXXXXXXXXXXXXXX-
 *          /XXX(       U     )        XXXXXXX\
 *        /XXXXX(              )--_  XXXXXXXXXXX\
 *       /XXXXX/ (      O     )   XXXXXX   \XXXXX\
 *       XXXXX/   /            XXXXXX   \__ \XXXXX----
 *       XXXXXX__/          XXXXXX         \__----  -
-*--___  XXX__/          XXXXXX      \__         ---
 * --  --__/   ___/\  XXXXXX            /  ___---=
 *   -_    ___/    XXXXXX              '--- XXXXXX
 *     --\/XXX\ XXXXXX                      /XXXXX
 *       \XXXXXXXXX                        /XXXXX/
 *        \XXXXXX                        _/XXXXX/
 *          \XXXXX--__/              __-- XXXX/
 *           --XXXXXXX---------------  XXXXX--
 *              \XXXXXXXXXXXXXXXXXXXXXXXX-
 *                --XXXXXXXXXXXXXXXXXX-
 *          * * * * * who ya gonna call? * * * * *
 *
 *    ______       _   _               _
 *    | ___ \     | | | |             | |
 *    | |_/ / ___ | |_| |__  _   _ ___| |_ ___ _ __ ___
 *    | ___ \/ _ \| __| '_ \| | | / __| __/ _ \ '__/ __|
 *    | |_/ / (_) | |_| |_) | |_| \__ \ ||  __/ |  \__ \
 *    \____/ \___/ \__|_.__/ \__,_|___/\__\___|_|  |___/
 *
 */

#ifndef RUNNING_FRC_TESTS

int main() {
	std::shared_ptr<spdlog::logger> log = spdlog::stdout_color_mt("Main");

#ifdef SIMULATION
	if(std::strcmp(NT_REMOTE_SERVER,  "127.0.0.1") != 0){
			log->warn("Running with remote NT server {}!", NT_REMOTE_SERVER);
			NT_StartClient(NT_GetDefaultInstance(), NT_REMOTE_SERVER, NT_REMOTE_PORT);
	}
	NT_SetUpdateRate(NT_GetDefaultInstance(), 0.01); // Set update rate to quickest possible (Limited by NT)

	log->warn("Running in simulation mode!");
	return frc::StartRobot<SimulationRobot>();
#endif
	
	return frc::StartRobot<StayPufft>();
}

#endif