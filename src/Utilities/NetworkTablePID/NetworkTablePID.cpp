//
// Created by abiel on 11/3/21.
//

#include "NetworkTablePID.h"

NetworkTablePID::NetworkTablePID(const std::string &entryName, PIDConfig defaultValue,
                                 const std::function<void(const PIDConfig &)> &updateFunction) {
	table = nt::NetworkTableInstance::GetDefault().GetTable(entryName);
	prevDefinition = defaultValue;
	
	auto pEntry = table->GetEntry("P");
	auto iEntry = table->GetEntry("I");
	auto dEntry = table->GetEntry("D");
	
	pEntry.SetDouble(prevDefinition.p);
	iEntry.SetDouble(prevDefinition.i);
	dEntry.SetDouble(prevDefinition.d);
	
	notifier = std::make_unique<frc::Notifier>([&] {
		PIDConfig newConfig = prevDefinition;
		newConfig.p = pEntry.GetDouble(prevDefinition.p);
		newConfig.i = iEntry.GetDouble(prevDefinition.i);
		newConfig.d = dEntry.GetDouble(prevDefinition.d);
		if (newConfig != prevDefinition) {
			updateFunction(newConfig);
			prevDefinition = newConfig;
		}
	});
	
	notifier->StartPeriodic(updateRate);
}