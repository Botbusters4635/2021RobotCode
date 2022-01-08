//
// Created by andrew on 03/11/21.
//

#ifndef BOTBUSTERS_REBIRTH_INTERPOLATINGTABLE_H
#define BOTBUSTERS_REBIRTH_INTERPOLATINGTABLE_H

#include <iostream>
#include <map>
#include <vector>
#include <algorithm>

using namespace std;

class InterpolatingTable {
public:
	explicit InterpolatingTable();
	
	void setTableData(double x, double y);
	
	double getIntVel(double x);

private:
	
	double interpolVelocity(double x);
	
	map<double, double> interTable;
	
	bool written = false;
	
};


#endif //BOTBUSTERS_REBIRTH_INTERPOLATINGTABLE_H
