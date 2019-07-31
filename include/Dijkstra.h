//
// Created by sokol on 31.07.19.
//

#ifndef OSM_FAPRA_DIJKSTRA_H
#define OSM_FAPRA_DIJKSTRA_H

#include "Graph.h"

namespace osmfapra {
class Dijkstra {
private:
	Graph& graph;
	std::vector<Distance> costs;
	std::vector<bool> visited;
	class PQElement {
	public:
		NodeId id;
		Distance cost;
		bool operator<(const PQElement& rhs) const {
			return cost > rhs.cost;
		}
		PQElement(size_t id, size_t cost): id(id), cost(cost) {}
	};

public:
	explicit Dijkstra(Graph & graph);
	Distance shortestDistance(NodeId source, NodeId target);
};
}

#endif //OSM_FAPRA_DIJKSTRA_H
