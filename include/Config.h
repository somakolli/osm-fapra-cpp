//
// Created by sokol on 29.07.19.
//

#ifndef OSM_FAPRA_CONFIG_H
#define OSM_FAPRA_CONFIG_H
#include <string>
#include <vector>
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
	template <typename Iter>
	Iter findHighway(std::string highwayValue);
	template <typename Iter>
	Iter findHighway(std::uint8_t highwayId);

public:
	explicit Config(const std::string& file);
	template <typename highway>
	uint8_t getMaxSpeed(highway highwayValue);
	template <typename highway>
	bool isValidHighway(highway highwayValue);
};
}

#endif //OSM_FAPRA_CONFIG_H
