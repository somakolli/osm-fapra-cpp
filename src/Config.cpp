//
// Created by sokol on 29.07.19.
//

#include "Config.h"
#include <fstream>
#include <algorithm>
#include <vector>

osmfapra::Config::Config(const std::string &file) {
	std::ifstream fileStream(file);
	if (fileStream.is_open()) {
		std::string value;
		std::uint8_t id;
		std::uint8_t maxSpeed;
		while (fileStream >> value >> id >> maxSpeed) {
			highwayInfos.emplace_back(HighwayInfo{id, value, maxSpeed});
		}
		fileStream.close();
	}
}
template <typename Iter>
Iter osmfapra::Config::findHighway(std::string highwayValue) {
	return std::find_if(highwayInfos.begin(), highwayInfos.end(), [&highwayValue](const HighwayInfo& x ){return x.highwayValue == highwayValue;});
}

template <typename Iter>
Iter osmfapra::Config::findHighway(std::uint8_t highwayId) {
	return std::find_if(highwayInfos.begin(), highwayInfos.end(), [&highwayId](const HighwayInfo& x ){return x.id == highwayId;});

}

template<typename highway>
uint8_t osmfapra::Config::getMaxSpeed(highway highwayValue) {
	return findHighway(std::move(highwayValue))->maxSpeed;
}

template<typename highway>
bool osmfapra::Config::isValidHighway(highway highwayValue) {
	return findHighway(highwayValue) != highwayInfos.end();
}
