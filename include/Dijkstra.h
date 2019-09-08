//
// Created by sokol on 31.07.19.
//

#ifndef OSM_FAPRA_DIJKSTRA_H
#define OSM_FAPRA_DIJKSTRA_H

#include "Graph.h"

namespace osmfapra {
class CostNode {
public:
	NodeId id;
	Distance cost;
	bool operator<(const CostNode& rhs) const {
		return cost > rhs.cost;
	}
	CostNode(size_t id, size_t cost): id(id), cost(cost) {}
};

class Dijkstra {
private:
	Graph& graph;
	std::vector<Distance> costs;
	std::vector<bool> visited;


public:
	explicit Dijkstra(Graph & graph);
	Distance shortestDistance(NodeId source, NodeId target);
};
}

#endif //OSM_FAPRA_DIJKSTRA_H
