//
// Created by sokol on 02.08.19.
//

#include <CHDijkstra.h>
#include <queue>
#include <algorithm>

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
				visited[currentEdge.target] = true;
				costs[currentEdge.target] = addedCost;
				queue.emplace(currentEdge.target, addedCost);
			}
		}
	}
	return settledNodes;
}

osmfapra::Distance osmfapra::CHDijkstra::shortestDistance(osmfapra::NodeId source, osmfapra::NodeId target) {
	auto upwardVec = shortestDistance(source, graph);
	auto downwardVec = shortestDistance(target, backGraph);
	std::map<NodeId, Distance > upwardMap;
	for(auto costNode : upwardVec) {
		upwardMap[costNode.id] = costNode.cost;
	}

	auto c = std::numeric_limits<Distance >::max();
	for(auto costNode : downwardVec) {
		if(upwardMap.find(costNode.id)!= upwardMap.end()) {
			auto considered_cost = upwardMap[costNode.id] + costNode.cost;
			if(considered_cost < c)
				c = considered_cost;
		}
	}
	return c;
}
