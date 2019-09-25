//
// Created by sokol on 31.07.19.
//
#include <queue>
#include <cmath>
#include "../include/Dijkstra.h"
template <typename Graph>
osmfapra::Distance osmfapra::Dijkstra<Graph>::shortestDistance(osmfapra::NodeId source, osmfapra::NodeId target) {

	if(target > graph.nodes.size())
		return std::numeric_limits<Distance >::max();
	if(source > graph.nodes.size())
		return std::numeric_limits<Distance >::max();
	if(source == lastSource && costs[target] < std::numeric_limits<Distance>::max()) {
		return costs[target];
	}
	lastSource = source;
	// clean up
	for(auto nodeId: visited) {
		costs[nodeId] = std::numeric_limits<Distance>::max();
	}
	visited.emplace_back(source);
	visited.emplace_back(target);
	previousNode.clear();
	costs[source] = 0;
	std::priority_queue<CostNode> queue;
	queue.emplace(source, 0);
	while(!queue.empty()) {
		auto currentNode = queue.top();
		queue.pop();
		if(currentNode.id == target) {
			return currentNode.cost;
		}
		auto begin = graph.offset[currentNode.id];
		auto end = graph.offset[currentNode.id+1];
		for(auto i = begin; i < end; ++i) {
			auto& currentEdge = graph.edges[i];
			Distance addedCost = currentNode.cost + currentEdge.distance;
			if(costs[currentEdge.target] > addedCost) {
				visited.emplace_back(currentEdge.target);
				costs[currentEdge.target] = addedCost;
				previousNode[currentEdge.target] = currentEdge.source;
				queue.emplace(currentEdge.target, addedCost);
			}
		}
	}
	return costs[target];
}
template <typename Graph>
osmfapra::Distance osmfapra::Dijkstra<Graph>::shortestDistance(osmfapra::LatLng source, osmfapra::LatLng target) {
	auto sourceId = osmfapra::Graph::getClosestNode<Graph>(graph, source);
	auto targetId = osmfapra::Graph::getClosestNode<Graph>(graph, target);
	return shortestDistance(sourceId, targetId);
}
template <typename Graph>
std::vector<osmfapra::Distance> osmfapra::Dijkstra<Graph>::shortestDistance(osmfapra::LatLng source, const std::vector<osmfapra::LatLng>& targets) {
	std::vector<osmfapra::Distance> distances;
	auto sourceId = osmfapra::Graph::getClosestNode<Graph>(graph, source);
	for (auto target: targets) {
		auto targetId = osmfapra::Graph::getClosestNode<Graph>(graph, target);
		distances.emplace_back(shortestDistance(sourceId, targetId));
	}
	return distances;
}

template <typename Graph>
std::vector<osmfapra::NodeId> osmfapra::Dijkstra<Graph>::shortestPath(NodeId source, NodeId target) {
	std::vector<NodeId> path;
	path.emplace_back(target);
	auto currentNode = target;
	while(previousNode[currentNode] != source) {
		currentNode = previousNode[currentNode];
		path.emplace_back(currentNode);
	}
	path.emplace_back(source);
	return path;
}

template<typename Graph>
bool
osmfapra::Dijkstra<Graph>::checkInPath(osmfapra::NodeId source, osmfapra::NodeId target, osmfapra::NodeId v) {
	auto currentNode = target;
	int i = 0;
	while(previousNode[currentNode] != source) {
		currentNode = previousNode[currentNode];
		++i;
	}
	//this means that the path consists of 3 nodes: source->v->target
	return i == 1 && previousNode[target] == v;
}

template<typename Graph>
std::vector<osmfapra::Distance>
osmfapra::Dijkstra<Graph>::multiSourceMultiTarget(const std::vector<osmfapra::NodeId> &sources,
												  const std::vector<osmfapra::NodeId> &targets) {
	return std::vector<osmfapra::Distance>();
}

template<typename Graph>
std::vector<osmfapra::Distance>
osmfapra::Dijkstra<Graph>::multiSourceMultiTarget(const std::vector<osmfapra::LatLng> &sources,
												  const std::vector<osmfapra::LatLng> &targets) {
	return std::vector<osmfapra::Distance>();
}


template class osmfapra::Dijkstra<osmfapra::CHGraph>;
template class osmfapra::Dijkstra<osmfapra::Graph>;

