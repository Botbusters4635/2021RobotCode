//
// Created by abiel on 12/25/19.
//

#include "Control/Path/PathParserPF1.h"
#include <iostream>
#include <fstream>
#include <sstream>

Path PathParserPF1::loadPath(const std::string &filePath) {
	std::ifstream file(filePath);
	std::vector<RobotPose2D> poses;
	
	std::string currentLine;
	
	//Skip header
	std::getline(file, currentLine);
	
	size_t lineCount = 1;
	while (file.good()) {
		std::getline(file, currentLine);
		lineCount++;
		
		std::stringstream lineStream(currentLine);
		
		std::string buffer;
		std::vector<std::string> row;
		while (lineStream.good()) {
			std::getline(lineStream, buffer, ',');
			row.emplace_back(buffer);
		}
		
		if (row.size() != 8) {
			throw std::runtime_error("Invalid file: " + filePath + " error in line: " + std::to_string(lineCount));
		}
		
		poses.emplace_back(std::stod(row[1]), std::stod(row[2]), Rotation2D(std::stod(row[7])));
	}
	
	file.close();
	
	return Path(poses);
}