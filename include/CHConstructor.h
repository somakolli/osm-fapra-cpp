//
// Created by sokol on 19.09.19.
//

#ifndef OSM_FAPRA_CHCONSTRUCTOR_H
#define OSM_FAPRA_CHCONSTRUCTOR_H


#include <vector>
#include "Graph.h"
#include "GraphBuilder.h"
#include "set"
#include "list"
#include "Dijkstra.h"

namespace osmfapra {
constexpr Distance MAX_DISTANCE = std::numeric_limits<Distance>::max();
class CHConstructor {
using Children = std::pair<EdgeId, EdgeId>;
private:
	CHGraph& backGraph;
	Graph& inputGraph;
	CHGraph& outputGraph;
	std::vector<NodeId> visited;
	std::vector<Distance> costs;
	uint32_t rounds;
public:
	CHConstructor(Graph &inputGraph, CHGraph &outputGraph, CHGraph &backGraph, uint32_t rounds);

private:
	class EdgeDifferenceNode {
	public:
		NodeId id;
		double edgeDifference;

		bool operator<(const EdgeDifferenceNode& rhs) const {
			return edgeDifference > rhs.edgeDifference;
		}
		EdgeDifferenceNode(NodeId id, double edgeDifference): id(id), edgeDifference(edgeDifference) {}
	};
	void constructCh();
	template <typename Graph>
	void buildIndependentSet(const Graph& graph, size_t count, std::vector<NodeId>& independentSet, std::size_t initNodeSize, const Graph& backgraph, NodeId firstNode);
	template <typename Graph>
	double getAverageEdgeDifference(const Graph &graph, const Graph& backGraph, size_t initNodeSize, osmfapra::NodeId startNode);
	template <typename Graph>
	int64_t getShortCuts(const Graph& graph, const Graph& myBackGraph, NodeId v, std::vector<osmfapra::CHEdgeWithId>* edgesToBeAdded = NULL);
	template <typename Graph>
	Distance calcDistanceLimit(const Graph& graph, const Graph& backGraph, NodeId source, NodeId v, NodeId target);
	void shortestDistance(const Graph& graph, std::vector<Distance>& myCosts, bool ignore, Distance radius, NodeId source, NodeId v, std::set<NodeId>& targets, std::vector<NodeId>& visitedWithoutV);

private:

};
}


#endif //OSM_FAPRA_CHCONSTRUCTOR_H
