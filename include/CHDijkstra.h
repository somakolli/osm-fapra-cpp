//
// Created by sokol on 02.08.19.
//

#ifndef OSM_FAPRA_CHDIJKSTRA_H
#define OSM_FAPRA_CHDIJKSTRA_H

#include <Dijkstra.h>
#include <set>
namespace osmfapra {
class CHDijkstra {
private:
	std::vector<bool> visited;
	std::vector<Distance> costs;
public:
	CHGraph& graph;
	CHGraph& backGraph;
	CHDijkstra(CHGraph& graph, CHGraph& backGraph);
	std::vector<CostNode> shortestDistance(NodeId source, CHGraph& graph);
	osmfapra::Distance shortestDistance(NodeId source, NodeId target);
	osmfapra::Distance shortestDistance(Lat lat1, Lng lng1, Lat lat2, Lng lng2);

};
}


#endif //OSM_FAPRA_CHDIJKSTRA_H
