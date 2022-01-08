//
// Created by andrew on 03/11/21.
//

#include "InterpolatingTable.h"

InterpolatingTable::InterpolatingTable() = default;

void InterpolatingTable::setTableData(double x, double y) {
	interTable[x] = y;
	written = true;
}

double InterpolatingTable::getIntVel(double x) {
	
	if (written) {
		auto it = interTable.find(x);
		if (it == interTable.end()) {
			return interpolVelocity(x);
		} else {
			return interTable[x];
		}
	} else {
		return 0.0;
	}
	
}

double InterpolatingTable::interpolVelocity(double x) {
	
	vector<double> maxs;
	vector<double> mins;
	for (auto &it: interTable) {
		if (it.first > x) {
			maxs.emplace_back(it.first);
		} else if (it.first < x) {
			mins.emplace_back(it.first);
		}
	}
	
	double predict = 0.0;
	
	double x1 = mins.back();
	double y1 = interTable[x1];
	double x2 = maxs.front();
	double y2 = interTable[x2];
	
	predict = y1 + ((x - x1) * ((y2 - y1) / (x2 - x1)));
	
	return predict;
}
