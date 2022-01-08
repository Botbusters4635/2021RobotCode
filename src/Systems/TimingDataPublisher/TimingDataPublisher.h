//
// Created by abiel on 10/13/19.
//

#ifndef BOTBUSTERSREBIRTH_TIMINGDATAPUBLISHER_H
#define BOTBUSTERSREBIRTH_TIMINGDATAPUBLISHER_H

#include <Core/EctoModule/System.h>
#include <Core/EctoModule/SystemHandler.h>

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>

//TODO Publish init times
class TimingDataPublisher : public System {
public:
	TimingDataPublisher(std::shared_ptr<SystemHandler> handler,
	                    const std::string &dataPublisherName = "TimingDataPublisher");
	
	void robotUpdate() override;

private:
	nt::NetworkTableInstance ntInstance = nt::NetworkTableInstance::GetDefault();
	std::shared_ptr<nt::NetworkTable> table;
	
	std::shared_ptr<SystemHandler> handler;
};


#endif //BOTBUSTERSREBIRTH_TIMINGDATAPUBLISHER_H
