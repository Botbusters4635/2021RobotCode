//
// Created by abiel on 10/13/19.
//

#include "TimingDataPublisher.h"

TimingDataPublisher::TimingDataPublisher(std::shared_ptr<SystemHandler> handler, const std::string &dataPublisherName)
		: System(dataPublisherName) {
	this->handler = handler;
	table = ntInstance.GetTable("TimingDataPublisher");
}

void TimingDataPublisher::robotUpdate() {
	for (const auto data: handler->getTimingData()) {
		const std::string systemName = data.first;
		table->GetEntry(fmt::format("{}/UpdateRobot", systemName)).SetDouble(data.second.updateRobotTime);
		table->GetEntry(fmt::format("{}/UpdateDisabled", systemName)).SetDouble(data.second.updateRobotTime);
		table->GetEntry(fmt::format("{}/MaxUpdateRobot", systemName)).SetDouble(data.second.updateRobotTime);
		table->GetEntry(fmt::format("{}/MaxUpdateDisabled", systemName)).SetDouble(data.second.updateRobotTime);
	}
}