//
// Created by sokol on 29.07.19.
//

#ifndef OSM_FAPRA_CONFIG_H
#define OSM_FAPRA_CONFIG_H
#include <string>
#include <vector>
#include <iostream>

namespace osmfapra{
class Config {
private:
	std::string file;
	struct HighwayInfo {
		uint8_t id;
		std::string highwayValue;
		uint8_t maxSpeed;
	};
	std::vector<HighwayInfo> highwayInfos;

public:
	explicit Config(const std::string& file);

	bool isValidHighway(const std::string& highwayValue){
		bool isValid = false;
		for(const auto &highwayInfo: highwayInfos) {
			if(highwayInfo.highwayValue == highwayValue) {
				isValid = true;
				break;
			}
		}
		return isValid;
	};
};
}

#endif //OSM_FAPRA_CONFIG_H
