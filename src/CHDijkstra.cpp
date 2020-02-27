//
// Created by sokol on 02.08.19.
//

#include <queue>
#include <algorithm>
#include <cmath>
#include <chrono>
#include "../include/CHDijkstra.h"
#include "../include/Graph.h"
#include "../include/Dijkstra.h"


osmfapra::CHDijkstra::CHDijkstra(osmfapra::CHGraph &graph, osmfapra::CHGraph& backGraph): graph(graph), backGraph(backGraph) {
	costs.reserve(graph.nodes.size());
	visited.reserve(graph.nodes.size());
	previousNode.reserve(graph.nodes.size());
	while (visited.size() < graph.nodes.size()) {
		visited.emplace_back(false);
		costs.emplace_back(std::numeric_limits<Distance >::max());
		previousNode.emplace_back(std::numeric_limits<NodeId>::max());
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
				previousNode[currentEdge.target] = currentEdge.source;
				queue.emplace(currentEdge.target, addedCost);
			}
		}
	}
	return settledNodes;
}

osmfapra::Distance osmfapra::CHDijkstra::shortestDistance(osmfapra::NodeId source, osmfapra::NodeId target) {
	const auto& upwardVec = shortestDistance(source, graph);
	const auto& downwardVec = shortestDistance(target, backGraph);
	NodeId topNode = 0;
	return findDistance(upwardVec, downwardVec, topNode);
}

osmfapra::Distance osmfapra::CHDijkstra::findDistance(const std::vector<osmfapra::CostNode> &upwardVec, const std::vector<osmfapra::CostNode> &downwardVec, NodeId& topNode) {
	std::map<osmfapra::NodeId, osmfapra::Distance> upwardMap;
	for (auto costNode : upwardVec) {
		upwardMap[costNode.id] = costNode.cost;
	}

	auto c = std::numeric_limits<osmfapra::Distance>::max();
	for (auto costNode : downwardVec) {
		if (upwardMap.find(costNode.id) != upwardMap.end()) {
			auto considered_cost = upwardMap[costNode.id] + costNode.cost;
			if (considered_cost < c) {
				c = considered_cost;
				topNode = costNode.id;
			}
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
		NodeId topNode;
		distanceVec.push_back(findDistance(upwardVec, downwardVec, topNode));
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
			NodeId id;
			distanceVec.emplace_back(findDistance(upwardSettledNodes, downwardSettledNodes, id));
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
	std::vector<NodeId> path;
	const auto& upwardVec = shortestDistance(source, graph);
	const auto& downwardVec = shortestDistance(target, backGraph);
	NodeId topNode = 0;
	Distance d = findDistance(upwardVec, downwardVec, topNode);
	if(d == MAX_DISTANCE || d == 0)
		return path;
	const auto& downPath = Dijkstra<CHGraph>::shortestPath(target, topNode, previousNode);
	shortestDistance(source, graph);
	// run dijkstra again to get previousNode correct
	const auto& reverseUpPath = Dijkstra<CHGraph>::shortestPath(source, topNode, previousNode);

	// get correct up path
	std::vector<NodeId> upPath;
	for(int i = reverseUpPath.size() - 1; i >= 0; --i) {
		upPath.emplace_back(reverseUpPath[i]);
	}

	std::vector<CHEdge> edges;
	//get all up edges
	for(int i = 0; i < upPath.size() - 1; ++i) {
		auto& begin = graph.offset[upPath[i]];
		auto& end = graph.offset[upPath[i] + 1];
		for(int j = begin; j < end; ++j) {
			const auto& edge = graph.edges[j];
			if(edge.target == upPath[i+1]){
				edges.emplace_back(edge);
			}
		}
	}

	//get all downEdges
	for(int i = 0; i < downPath.size() - 1; ++i) {
		auto& begin = graph.offset[downPath[i]];
		auto& end = graph.offset[downPath[i] + 1];
		for(int j = begin; j < end; ++j) {
			const auto& edge = graph.edges[j];
			if(edge.target == downPath[i+1]) {
				edges.emplace_back(edge);
			}
		}
	}
	for(const auto& edge: edges) {
		for(const auto& id : graph.getPathFromShortcut(edge)) {
			path.emplace_back(id);
		}
	}
	return path;
}

std::vector<osmfapra::LatLng> osmfapra::CHDijkstra::shortestPath(osmfapra::LatLng source, osmfapra::LatLng target) {
	NodeId sourceId = Graph::getClosestNode<CHGraph>(graph, source);
	NodeId targetId = Graph::getClosestNode<CHGraph>(graph, target);
	std::vector<osmfapra::LatLng> path;

	for(const auto& id: shortestPath(sourceId, targetId)){
		const Node& node = graph.nodes[id];
		path.emplace_back(node.lat, node.lng);
	}
	return path;
}
