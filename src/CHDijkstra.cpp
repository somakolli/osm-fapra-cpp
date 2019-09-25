//
// Created by sokol on 02.08.19.
//

#include <queue>
#include <algorithm>
#include <cmath>
#include "../include/CHDijkstra.h"
#include "../include/Graph.h"
#include "../include/Dijkstra.h"


osmfapra::CHDijkstra::CHDijkstra(osmfapra::CHGraph &graph, osmfapra::CHGraph& backGraph): graph(graph), backGraph(backGraph) {
	costs.reserve(graph.nodes.size());
	visited.reserve(graph.nodes.size());
	while (visited.size() < graph.nodes.size()) {
		visited.emplace_back(false);
	}
	while (costs.size() < graph.nodes.size()) {
		costs.emplace_back(std::numeric_limits<Distance >::max());
	}
}

std::vector<osmfapra::CostNode > osmfapra::CHDijkstra::shortestDistance(osmfapra::NodeId source, CHGraph& graph){
	std::vector<osmfapra::CostNode > settledNodes;
	if(source > graph.nodes.size()) {
		return settledNodes;
	}

	// clean up
	for(auto nodeId : visited) {
		costs[nodeId] = std::numeric_limits<Distance>::max();
	}
	visited.emplace_back(source);
	costs[source] = 0;
	std::priority_queue<CostNode> queue;
	queue.emplace(source, 0);
	while(!queue.empty()) {
		auto currentNode = queue.top();
		queue.pop();
		if(costs[currentNode.id] == currentNode.cost)
			settledNodes.emplace_back(currentNode);
		auto begin = graph.offset[currentNode.id];
		auto end = graph.offset[currentNode.id+1];
		for(auto i = begin; i < end; ++i) {
			auto& currentEdge = graph.edges[i];
			if(graph.nodes[currentEdge.target].level < graph.nodes[currentEdge.source].level) {
				continue;
			}
			Distance addedCost = currentNode.cost + currentEdge.distance;
			if(costs[currentEdge.target] > addedCost) {
				visited.emplace_back(currentEdge.target);
				costs[currentEdge.target] = addedCost;
				queue.emplace(currentEdge.target, addedCost);
			}
		}
	}
	return settledNodes;
}

osmfapra::Distance osmfapra::CHDijkstra::shortestDistance(osmfapra::NodeId source, osmfapra::NodeId target) {
	const auto& upwardVec = shortestDistance(source, graph);
	const auto& downwardVec = shortestDistance(target, backGraph);
	return findDistance(upwardVec, downwardVec);
}

osmfapra::Distance osmfapra::CHDijkstra::findDistance(const std::vector<osmfapra::CostNode> &upwardVec, const std::vector<osmfapra::CostNode> &downwardVec) {
	std::map<osmfapra::NodeId, osmfapra::Distance> upwardMap;
	for (auto costNode : upwardVec) {
		upwardMap[costNode.id] = costNode.cost;
	}

	auto c = std::numeric_limits<osmfapra::Distance>::max();
	for (auto costNode : downwardVec) {
		if (upwardMap.find(costNode.id) != upwardMap.end()) {
			auto considered_cost = upwardMap[costNode.id] + costNode.cost;
			if (considered_cost < c)
				c = considered_cost;
		}
	}
	return c;
}

osmfapra::Distance
osmfapra::CHDijkstra::shortestDistance(LatLng source, LatLng target) {
	NodeId sourceId = Graph::getClosestNode<CHGraph>(graph, source);
	NodeId targetId = Graph::getClosestNode<CHGraph>(graph, target);
	return shortestDistance(sourceId, targetId);
}

std::vector<osmfapra::Distance> osmfapra::CHDijkstra::shortestDistance(const osmfapra::LatLng& source , std::vector<osmfapra::LatLng>& targets){
	std::vector<Distance > distanceVec;
	NodeId sourceId = Graph::getClosestNode<CHGraph>(graph, source);
	auto upwardVec = shortestDistance(sourceId, graph);
	for(auto target : targets) {
		NodeId targetId = Graph::getClosestNode<CHGraph>(graph, target);
		const auto& downwardVec = shortestDistance(targetId, backGraph);
		distanceVec.push_back(findDistance(upwardVec, downwardVec));
	}
	return distanceVec;
}

std::vector<osmfapra::Distance> osmfapra::CHDijkstra::multiSourceMultiTarget(const std::vector<osmfapra::NodeId>& sources, const std::vector<osmfapra::NodeId>& targets) {
	std::vector<Distance > distanceVec;
	std::vector<std::vector<CostNode>> upWardSettledNodesVec;
	std::vector<std::vector<CostNode>> downwardSettledNodesVec;
	upWardSettledNodesVec.reserve(sources.size());
	for(auto& source: sources) {
		upWardSettledNodesVec.emplace_back(shortestDistance(source, graph));
	}
	downwardSettledNodesVec.reserve(targets.size());
	for(auto& target: targets) {
		downwardSettledNodesVec.emplace_back(shortestDistance(target, backGraph));
	}
	std::cout << downwardSettledNodesVec.size() << std::endl;
	for(auto& upwardSettledNodes: upWardSettledNodesVec) {
		for(auto& downwardSettledNodes: downwardSettledNodesVec) {
			distanceVec.emplace_back(findDistance(upwardSettledNodes, downwardSettledNodes));
		}
	}
	return distanceVec;
}

std::vector<osmfapra::Distance> osmfapra::CHDijkstra::multiSourceMultiTarget(const std::vector<osmfapra::LatLng>& sources, const std::vector<osmfapra::LatLng>& targets) {
	std::vector<NodeId> idSources;
	std::vector<NodeId> idTargets;
	idSources.reserve(sources.size());
	idTargets.reserve(targets.size());
	for(auto source: sources) {
		idSources.emplace_back(Graph::getClosestNode<CHGraph>(graph, source));
	}
	for(auto target: targets) {
		idTargets.emplace_back(Graph::getClosestNode<CHGraph>(graph, target));
	}
	std::cout << "sources: " << sources.size() << std::endl;
	std::cout << "targets: " << targets.size() << std::endl;

	return multiSourceMultiTarget(idSources, idTargets);
}
std::vector<osmfapra::NodeId> osmfapra::CHDijkstra::shortestPath(osmfapra::NodeId source, osmfapra::NodeId target) {
	return std::vector<osmfapra::NodeId>();
}
