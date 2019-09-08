//
// Created by sokol on 31.07.19.
//
#include <queue>
#include "Dijkstra.h"

osmfapra::Dijkstra::Dijkstra(osmfapra::Graph &graph) : graph(graph) {
	costs.reserve(graph.nodes.size());
	visited.reserve(graph.nodes.size());
	while (visited.size() < graph.nodes.size()) {
		visited.emplace_back(false);
	}
	while (costs.size() < graph.nodes.size()) {
		costs.emplace_back(std::numeric_limits<Distance >::max());
	}
}

osmfapra::Distance osmfapra::Dijkstra::shortestDistance(osmfapra::NodeId source, osmfapra::NodeId target) {
	if(target > graph.nodes.size())
		return std::numeric_limits<Distance >::max();
	if(source > graph.nodes.size())
		return std::numeric_limits<Distance >::max();

	// clean up
	for(auto i = 0; i < visited.size(); ++i) {
		if(visited[i]){
			costs[i] = std::numeric_limits<Distance>::max();
		}
	}
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
				visited[currentEdge.target] = true;
				costs[currentEdge.target] = addedCost;
				queue.emplace(currentEdge.target, addedCost);
			}
		}
	}
	return costs[target];
}
