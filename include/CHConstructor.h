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
private:
	CHGraph& backGraph;
	Graph& inputGraph;
	CHGraph& outputGraph;
	std::vector<NodeId> visited;
	std::vector<Distance> costs;
	std::vector<Distance> costsWithoutV;
public:
	CHConstructor(Graph &inputGraph, CHGraph &outputGraph, CHGraph &backGraph);

private:
	class EdgeDifferenceNode {
	public:
		NodeId id;
		int64_t edgeDifference;

		bool operator<(const EdgeDifferenceNode& rhs) const {
			return edgeDifference > rhs.edgeDifference;
		}
		EdgeDifferenceNode(NodeId id, int64_t edgeDifference): id(id), edgeDifference(edgeDifference) {}
	};
	void constructCh();
	template <typename Graph>
	void buildIndependentSet(const Graph& graph, size_t count, std::vector<NodeId>& independentSet, std::size_t initNodeSize, Graph& backgraph);
	template <typename Graph, typename DijkstraGraph>
	int64_t computeEdgeDifference(const Graph& graph, const Graph& backGraph, NodeId node, Dijkstra<DijkstraGraph>& dijkstra,
								  std::vector<osmfapra::EdgeId >& edgesToBeDeleted, std::vector<osmfapra::CHEdge>& edgesToBeAdded);
	template <typename Graph>
	int64_t getShortCuts(const Graph& graph, const Graph& backGraph, NodeId v, std::vector<osmfapra::CHEdge>& edgesToBeAdded);
	template <typename Graph>
	Distance calcDistanceLimit(const Graph& graph, const Graph& backGraph, NodeId source, NodeId v, NodeId target);
	void shortestDistance(const Graph& graph, std::vector<Distance>& myCosts, bool ignore, Distance radius, NodeId source, NodeId v, std::set<NodeId>& targets);

private:
	template <typename T, typename IndexType>
	void removeFromVec(std::vector<T>& vec, IndexType index) {
		if(vec.empty())
			return;
		vec[index] = vec.back();
		vec.pop_back();
	}
};
}


#endif //OSM_FAPRA_CHCONSTRUCTOR_H
