//
// Created by sokol on 02.08.19.
//

#ifndef OSM_FAPRA_CHDIJKSTRA_H
#define OSM_FAPRA_CHDIJKSTRA_H

#include <set>
#include "Graph.h"
#include "Dijkstra.h"

namespace osmfapra {
class CHDijkstra {
private:
	std::vector<NodeId> visited;
	std::vector<Distance> costs;
	std::vector<NodeId> previousNode;
	Distance findDistance(const std::vector<osmfapra::CostNode> &upwardVec, const std::vector<osmfapra::CostNode> &downwardVec, NodeId& topNode);
public:
	CHGraph& graph;
	CHGraph& backGraph;
	CHDijkstra(CHGraph& graph, CHGraph& backGraph);
	std::vector<CostNode> shortestDistance(NodeId source, CHGraph& graph);
	osmfapra::Distance shortestDistance(NodeId source, NodeId target);
	osmfapra::Distance shortestDistance(LatLng source , LatLng target);
	std::vector<osmfapra::Distance> shortestDistance(const LatLng& source , std::vector<LatLng>& targets);
	std::vector<osmfapra::Distance> multiSourceMultiTarget(const std::vector<NodeId>& sources, const std::vector<NodeId>& targets);
	std::vector<osmfapra::Distance> multiSourceMultiTarget(const std::vector<LatLng>& sources, const std::vector<LatLng>& targets);
	std::vector<NodeId> shortestPath(NodeId source, NodeId target);
	std::vector<osmfapra::LatLng> shortestPath(osmfapra::LatLng source, osmfapra::LatLng target);
};
}


#endif //OSM_FAPRA_CHDIJKSTRA_H
