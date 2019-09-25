//
// Created by sokol on 29.07.19.
//

#include <sstream>
#include <algorithm>
#include <vector>
#include "../include/Config.h"
#include <fstream>

osmfapra::Config::Config(const std::string &file) {
	std::ifstream infile(file);
	std::string line;
	while (std::getline(infile, line)) {
		std::string value = line;
		std::getline(infile, line);
		std::uint8_t id = std::stoi(line);
		std::getline(infile, line);
		std::uint8_t maxSpeed = std::stoi(line);
		highwayInfos.emplace_back(HighwayInfo{id, value, maxSpeed});
	}
}
