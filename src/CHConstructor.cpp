//
// Created by sokol on 19.09.19.
//


#include <assert.h>
#include "../include/Graph.h"
#include "../include/CHConstructor.h"
#include "../include/Dijkstra.h"
#include "unordered_map"
#include "list"
#include "queue"
#include "chrono"
#define function Print(string) = std::cout << string << '\n'
#define var DEBUG = 0;

osmfapra::CHConstructor::CHConstructor(osmfapra::Graph &inputGraph, osmfapra::CHGraph &outputGraph, CHGraph &backGraph) : inputGraph(
		inputGraph), outputGraph(outputGraph), backGraph(backGraph) {
	costs.reserve(inputGraph.nodes.size());
	while(costs.size() <= inputGraph.nodes.size()) {
		costs.emplace_back(MAX_DISTANCE);
		costsWithoutV.emplace_back(MAX_DISTANCE);
	}
	constructCh();
	std::cout << "ch constructed!" << std::endl;
}

void osmfapra::CHConstructor::constructCh() {
	osmfapra::Graph myBackGraph;

	GraphBuilder::buildBackGraph(inputGraph, myBackGraph);
	GraphBuilder::buildOffset(inputGraph);
	GraphBuilder::buildOffset(myBackGraph);
	size_t initNodeSize = inputGraph.nodes.size();
	Level l = 0;
	std::set<EdgeId > edgesAdded;
	while(!inputGraph.nodes.empty()) {
		std::cout << initNodeSize - inputGraph.nodes.size() << "/" << initNodeSize << std::endl;

		std::vector<NodeId> independentSet;
		buildIndependentSet(inputGraph, 10000, independentSet, initNodeSize, myBackGraph);
		std::set<osmfapra::EdgeId > allEdgesToBeDeleted;
		std::map<std::pair<NodeId, NodeId >,osmfapra::CHEdge> allEdgesToBeAdded;
		std::map<NodeId, std::vector<osmfapra::CHEdge>> edgesToBeAddedMap;
		std::vector<NodeId > nodesToContract;
		std::priority_queue<EdgeDifferenceNode> priorityQueue;
		auto start = std::chrono::steady_clock::now();
		std::vector<NodeId> nodesToBeContracted;
		for(const auto& id : independentSet) {
			std::vector<osmfapra::CHEdge> edgesToBeAdded;
			int64_t edgeDifference = getShortCuts(inputGraph, myBackGraph, id,
														   edgesToBeAdded);
			priorityQueue.push(EdgeDifferenceNode(id, edgeDifference));
			edgesToBeAddedMap[id] = edgesToBeAdded;
		}
		auto end = std::chrono::steady_clock::now();
		std::cout << "Elapsed time in milliseconds : "
				  << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
				  << " ms" << std::endl;
		size_t i = 0;
		while(!priorityQueue.empty() && i < 1000) {
			auto pqElement = priorityQueue.top();
			priorityQueue.pop();
			for(const auto& edge : edgesToBeAddedMap[pqElement.id]) {
				allEdgesToBeAdded[std::make_pair(edge.source, edge.target)] = (edge);
			}
			nodesToBeContracted.push_back(pqElement.id);
			++i;
		}
		std::sort(nodesToBeContracted.begin(), nodesToBeContracted.end());
		// contract nodes and gather edge indexes to be deleted
		std::vector<size_t> edgesToBeDeleted;
		for(int j = nodesToBeContracted.size() -1 ; j >= 0; --j) {
			NodeId id = nodesToBeContracted[j];
			auto& sourceEdgesBegin = myBackGraph.offset[id];
			auto& sourceEdgesEnd = myBackGraph.offset[id + 1];
			for(size_t k = sourceEdgesBegin; k<sourceEdgesEnd; ++k) {
				auto& source = myBackGraph.edges[k].target;
				auto& edgesBegin = inputGraph.offset[source];
				auto& edgesEnd = inputGraph.offset[source + 1];
				for(size_t m = edgesBegin; m < edgesEnd; ++m) {
					if(inputGraph.edges[m].target == id) {
						edgesToBeDeleted.push_back(m);
					}
				}
			}
			auto& targetEdgesBegin = inputGraph.offset[id];
			auto& targetEdgesEnd = inputGraph.offset[id+1];
			for(size_t k = targetEdgesBegin; k < targetEdgesEnd; ++k) {
				edgesToBeDeleted.push_back(k);
			}
			Node& node = inputGraph.nodes[inputGraph.indexMap[id]];
			outputGraph.nodes.emplace_back(CHNode{node.id, node.lat, node.lng, l});
			removeFromVec(inputGraph.nodes, inputGraph.indexMap[id]);
		}
		std::sort(edgesToBeDeleted.begin(), edgesToBeDeleted.end());
		for(int j = edgesToBeDeleted.size() -1 ; j >= 0; --j) {
			Edge& edge = inputGraph.edges[edgesToBeDeleted[j]];
			if(edgesAdded.find(std::make_pair(edge.source, edge.target)) == edgesAdded.end()) {
				outputGraph.edges.emplace_back(
						CHEdge{edge.source, edge.target, edge.distance, edge.maxSpeed, std::nullopt, std::nullopt});
				edgesAdded.emplace(std::make_pair(edge.source, edge.target));
			}
			removeFromVec(inputGraph.edges, edgesToBeDeleted[j]);
		}
		for(auto& edge : allEdgesToBeAdded) {
			inputGraph.edges.emplace_back(edge.second);
			if(edgesAdded.find(std::make_pair(edge.second.source, edge.second.target)) == edgesAdded.end()){
				outputGraph.edges.emplace_back(edge.second);
				edgesAdded.emplace(std::make_pair(edge.second.source, edge.second.target));
			}
		}
		++l;
		std::sort(inputGraph.edges.begin(), inputGraph.edges.end(), [](auto edge1, auto edge2) -> bool{
			return (edge1.source == edge2.source) ? edge1.target <= edge2.target : edge1.source < edge2.source;
		});
		GraphBuilder::sortNodes(inputGraph);

		GraphBuilder::buildOffset(inputGraph);
		GraphBuilder::buildBackGraph(inputGraph, myBackGraph);
		GraphBuilder::buildOffset(myBackGraph);
		GraphBuilder::buildIndexMap(inputGraph);
	}

	GraphBuilder::sortNodes(outputGraph);

	std::sort(outputGraph.edges.begin(), outputGraph.edges.end(), [](auto edge1, auto edge2) -> bool{
		return (edge1.source == edge2.source) ? edge1.target <= edge2.target : edge1.source < edge2.source;
	});

	GraphBuilder::buildOffset(outputGraph);
	GraphBuilder::buildBackGraph(outputGraph, backGraph);
	GraphBuilder::buildOffset(backGraph);
	std::cout << outputGraph << std::endl;
	std::cout << backGraph << std::endl;
#if DEBUG
	bool edgeAdded = false;
	int i = 0;
	for(auto& node: outputGraph.nodes) {
		std::cout << "node: " << i << '/' << outputGraph.nodes.size() << '\n';
		i++;

		if(edgeAdded) {
			std::sort(outputGraph.edges.begin(), outputGraph.edges.end(), [](auto edge1, auto edge2) -> bool{
				return (edge1.source == edge2.source) ? edge1.target <= edge2.target : edge1.source < edge2.source;
			});
			osmfapra::GraphBuilder::buildBackGraph(outputGraph, backGraph);
			osmfapra::GraphBuilder::buildOffset(backGraph);
			edgeAdded = false;
		}

		std::cout << "contracting node: " << node.id << '\n';
		std::cout << "forwardgraph \n" << outputGraph << '\n';
		std::cout << "backgraph \n" << backGraph << '\n';

		auto beginBackward = backGraph.offset[node.id];
		auto endBackward = backGraph.offset[node.id+1];
		for(auto i = beginBackward; i < endBackward; ++i) {
			auto beginForward = outputGraph.offset[node.id];
			auto endForward = outputGraph.offset[node.id+1];
			for(auto j = beginForward; j < endForward; ++j) {
				NodeId source = backGraph.edges[i].target;
				NodeId target = outputGraph.edges[j].target;
				auto distance = dijkstra.shortestDistance(source, target);
				if(distance >= std::numeric_limits<Distance>::max() || distance == 0) {
					break;
				}
				if(dijkstra.checkInPath(source, target, node.id)){
					const auto &edge = CHEdge{source, target, distance,
											  50, -1, -1};
					outputGraph.edges.emplace_back(edge);
					edgeAdded = true;
				}
				node.level = l;
				l++;
			}
		}
		GraphBuilder::buildOffset(outputGraph);
	}
#endif
	std::cout << "ch constructed!" << std::endl;

}

template<typename Graph>
void osmfapra::CHConstructor::buildIndependentSet(const Graph &graph, size_t count, std::vector<NodeId>& independentSet, std::size_t initNodeSize, Graph& backgraph) {
	std::vector<bool> markedNodes(initNodeSize, false);
	independentSet.clear();
	independentSet.reserve(count);
	for(size_t i = 0; i < graph.nodes.size(); ++i) {
		if(independentSet.size() > count) {
			break;
		}
		NodeId id = graph.nodes[i].id;
		if(!markedNodes[id]) {
			markedNodes[id] = true;
			independentSet.emplace_back(id);
			// mark all outgoing nodes
			for(auto j = graph.offset[id]; j < graph.offset[id+1]; ++j) {
				markedNodes[graph.edges[j].target] = true;
			}
			for(auto j = backgraph.offset[id]; j < backgraph.offset[id+1]; ++j) {
				markedNodes[backgraph.edges[j].target] = true;
			}
		}
	}

}

template<typename Graph, typename DijkstraGraph>
int64_t osmfapra::CHConstructor::computeEdgeDifference(const Graph &graph, const Graph& backGraph, osmfapra::NodeId nodeId, osmfapra::Dijkstra<DijkstraGraph>& dijkstra,
														std::vector<osmfapra::EdgeId >& edgesToBeDeleted, std::vector<osmfapra::CHEdge>& edgesToBeAdded) {
	auto beginBackward = backGraph.offset[nodeId];
	auto endBackward = backGraph.offset[nodeId+1];
	for(auto i = beginBackward; i < endBackward; ++i) {
		auto beginForward = graph.offset[nodeId];
		auto endForward = graph.offset[nodeId+1];
		for(auto j = beginForward; j < endForward; ++j) {
			auto &sourceEdge = backGraph.edges[i];
			NodeId source = sourceEdge.target;
			const auto &targetEdge = graph.edges[j];
			NodeId target = targetEdge.target;
			if(sourceEdge.deleted || targetEdge.deleted) {
				continue;
			}
			auto start = std::chrono::steady_clock::now();
			auto distance = dijkstra.shortestDistance(source, target);

			if(distance >= std::numeric_limits<Distance>::max() || distance == 0) {
				break;
			}
			if(dijkstra.checkInPath(source, target, nodeId)) {
				const auto& path = dijkstra.shortestPath(source, target);
				for (int k = path.size()-1; k > 0; --k) {
					edgesToBeDeleted.emplace_back(path[k], path[k-1]);
				}
				const auto &edge = CHEdge{source, target, distance,
										  50, -1, -1};
				edgesToBeAdded.emplace_back(edge);
			}
		}
	}
	return edgesToBeAdded.size() - edgesToBeDeleted.size();
}

template<typename Graph>
int64_t osmfapra::CHConstructor::getShortCuts
(const Graph& graph, const Graph& backGraph, osmfapra::NodeId v, std::vector<osmfapra::CHEdge>& edgesToBeAdded) {
	auto& sourceEdgesBegin = backGraph.offset[v];
	auto& sourceEdgesEnd = backGraph.offset[v + 1];
	if(sourceEdgesBegin == sourceEdgesEnd) {
		return 0;
	}
	auto& targetEdgesBegin = graph.offset[v];
	auto& targetEdgesEnd = graph.offset[v+1];
	size_t targetCount = 0;
	std::set<NodeId> targets;
	Distance maxDistanceToTarget = 0;
	for(auto j = targetEdgesBegin; j < targetEdgesEnd; ++j) {
		auto &edgeFromVToTarget = graph.edges[j];
		if(edgeFromVToTarget.distance > maxDistanceToTarget)
			maxDistanceToTarget = edgeFromVToTarget.distance;
		targets.insert(edgeFromVToTarget.target);
		++targetCount;
	}
	if(targetCount == 0) {
		return - (static_cast<int>(sourceEdgesEnd) - static_cast<int>(sourceEdgesBegin));
	}
	for(auto i = sourceEdgesBegin; i < sourceEdgesEnd; ++i) {

		// clean up
		auto &edgeFromVToSource = backGraph.edges[i];
		if(edgeFromVToSource.source == edgeFromVToSource.target) {
			break;
		}
		auto distanceFromVtoSource = edgeFromVToSource.distance;
		auto source = edgeFromVToSource.target;
		shortestDistance(graph, costs, true, MAX_DISTANCE, source, v, targets);
		//shortestDistance(graph, costsWithoutV, true, MAX_DISTANCE, source, v, targets);
		for(auto j = targetEdgesBegin; j < targetEdgesEnd; ++j) {
			auto& edgeFromVToTarget = graph.edges[j];
			auto& target = edgeFromVToTarget.target;
			auto distanceFromVToTarget = edgeFromVToTarget.distance;
			if(costs[target] > distanceFromVtoSource + distanceFromVToTarget && costsWithoutV[target] > distanceFromVtoSource + distanceFromVToTarget){
				//did not find an other path from source to target so the shortcut is necessary
				edgesToBeAdded.emplace_back(CHEdge{source, target, distanceFromVtoSource + distanceFromVToTarget, 50, std::nullopt, std::nullopt});
			}
		}
	}
	// edge difference: #addedShortcuts - #indicentEdges
	return static_cast<int>(edgesToBeAdded.size()) - (static_cast<int>(sourceEdgesEnd) - static_cast<int>(sourceEdgesBegin));
}

template<typename Graph>
osmfapra::Distance
osmfapra::CHConstructor::calcDistanceLimit(const Graph &graph, const Graph &backGraph, osmfapra::NodeId source,
										   osmfapra::NodeId v, osmfapra::NodeId target) {
	return 0;
}

void osmfapra::CHConstructor::shortestDistance(const osmfapra::Graph &graph, std::vector<osmfapra::Distance> &myCosts,
											   bool ignore, osmfapra::Distance radius, osmfapra::NodeId source,
											   osmfapra::NodeId v, std::set<NodeId>& targets) {

	for(auto nodeId : visited) {
		myCosts[nodeId] = MAX_DISTANCE;
	}
	visited.clear();
	myCosts[source] = 0;
	visited.emplace_back(source);
	std::priority_queue<CostNode> queue;
	queue.emplace(source, 0);
	uint targetsSettled = 0;
	while(!queue.empty()) {
		auto currentNode = queue.top();
		queue.pop();
		if(targets.find(currentNode.id) != targets.end())
			++targetsSettled;
		if(currentNode.cost > radius || targetsSettled == targets.size())
			break;
		if(myCosts[currentNode.id] > currentNode.cost) continue;
		auto begin = graph.offset[currentNode.id];
		auto end = graph.offset[currentNode.id+1];
		for(auto j = begin; j < end; ++j) {
			auto& currentEdge = graph.edges[j];
			if(currentEdge.target == v && ignore)
				continue;
			Distance addedCost = currentNode.cost + currentEdge.distance;
			if(myCosts[currentEdge.target] > addedCost) {
				visited.emplace_back(currentEdge.target);
				myCosts[currentEdge.target] = addedCost;
				queue.emplace(currentEdge.target, addedCost);
			}
		}
	}
}




