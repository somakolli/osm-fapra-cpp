//
// Created by sokol on 31.07.19.
//

#ifndef OSM_FAPRA_DIJKSTRA_H
#define OSM_FAPRA_DIJKSTRA_H

#include "Graph.h"
#include "GraphBuilder.h"

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
template <typename Graph>
class Dijkstra {
private:
	int lastSource = -1;
	Graph& graph;
	std::vector<Distance> costs;
	std::vector<NodeId> visited;
	std::vector<NodeId> previousNode;

public:
	explicit Dijkstra(Graph & graph) : graph(graph) {
		costs.reserve(graph.nodes.size());
		previousNode.reserve(graph.nodes.size());
		while (costs.size() < graph.nodes.size()) {
			costs.emplace_back(std::numeric_limits<Distance >::max());
			previousNode.emplace_back(std::numeric_limits<NodeId>::max());
		}
	}
	Distance shortestDistance(NodeId source, NodeId target);
	Distance shortestDistance(LatLng source, LatLng target);
	static std::vector<NodeId> shortestPath(NodeId source, NodeId target, const std::vector<NodeId>& myPreviousNode);
	std::vector<Distance> shortestDistance(LatLng source, const std::vector<LatLng>& targets);
	bool checkInPath(NodeId source, NodeId target, NodeId v);

	std::vector<osmfapra::Distance> multiSourceMultiTarget(const std::vector<NodeId>& sources, const std::vector<NodeId>& targets);
	std::vector<osmfapra::Distance> multiSourceMultiTarget(const std::vector<LatLng>& sources, const std::vector<LatLng>& targets);

	std::vector<osmfapra::LatLng> shortestPath(LatLng source, LatLng target);
};
}

#endif //OSM_FAPRA_DIJKSTRA_H
