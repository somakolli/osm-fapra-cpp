//
// Created by sokol on 27.07.19.
//

#ifndef OSM_FAPRA_GRID_H
#define OSM_FAPRA_GRID_H

#include <vector>
#include <map>
#include "Graph.h"
#include "GraphBuilder.h"
#include <cmath>

namespace osmfapra {
template< typename GridNode>
class Grid {
	using GridKey = std::pair<Lat, Lng>;
	std::vector<GridNode> nodes;
	int gridSizeY;
	int gridSizeX;
public:
	std::map<GridKey, std::vector<GridNode>> grid;
	Grid(const std::vector<GridNode> nodes, int gridSizeX, int gridSizeY): nodes(nodes), gridSizeX(gridSizeX), gridSizeY(gridSizeY) { };
	void buildGrid() {
		double divLat = 180.0f / static_cast<float>(gridSizeY);
		std::cout << 180.0f << "/" << gridSizeY << " = " << 180.0f / static_cast<float>(gridSizeX) << std::endl;
		double divLng = 360.0f / static_cast<float>(gridSizeX);
		for (const auto& node : nodes) {
			auto latPositionInGrid = floor(node.lat/ divLat);
			auto lngPositionInGrid = floor(node.lng / divLng);
			auto latPair = std::make_pair(latPositionInGrid, lngPositionInGrid);
			grid[latPair].emplace_back(std::move(node));
		}
	};
};
}



#endif //OSM_FAPRA_GRID_H
