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

osmfapra::CHConstructor::CHConstructor(osmfapra::Graph &inputGraph, osmfapra::CHGraph &outputGraph, CHGraph &backGraph, uint32_t rounds) : inputGraph(
		inputGraph), outputGraph(outputGraph), backGraph(backGraph), rounds(rounds) {
	costs.reserve(inputGraph.nodes.size());
	while(costs.size() <= inputGraph.nodes.size()) {
		costs.emplace_back(MAX_DISTANCE);
	}
	constructCh();
}

void osmfapra::CHConstructor::constructCh() {
	osmfapra::Graph myBackGraph;

	GraphBuilder::buildBackGraph(inputGraph, myBackGraph);
	GraphBuilder::buildOffset(inputGraph);
	GraphBuilder::buildOffset(myBackGraph);
	size_t initNodeSize = inputGraph.nodes.size();
	size_t initEdgeSize = inputGraph.edges.size();
	Level l = 0;
	std::map<EdgeId, CHEdgeWithId> allEdgesAdded;
	std::map<EdgeId, uint32_t> edgeToEdgeIndexMap;
	while(!inputGraph.nodes.empty()) {
		std::cout << "inputgraph edges size: " << inputGraph.edges.size() << std::endl;
		std::cout << "nodes contracted: " <<  initNodeSize - inputGraph.nodes.size() << "/" << initNodeSize << std::endl;
		std::cout << "Edges added: " << allEdgesAdded.size() << "/" << initEdgeSize << std::endl;
		std::vector<EdgeDifferenceNode> nodeMeanEdgeDifferenceVector;
		NodeId i = 0;
		std::srand(time(0));
		while(i < rounds){
			NodeId id = std::rand() % inputGraph.nodes.size();
			std::cout << "round: " << i << std::endl;
			nodeMeanEdgeDifferenceVector.emplace_back(id, getAverageEdgeDifference(inputGraph, myBackGraph, initNodeSize, id));
			++i;
		}
		NodeId bestStartNode = 0;
		double lowestEdgeDifference = std::numeric_limits<double >::max();
		for(const auto& edgeDifferenceNode: nodeMeanEdgeDifferenceVector) {
			std::cout << "Node id: " << edgeDifferenceNode.id << " average edge difference: " << edgeDifferenceNode.edgeDifference << std::endl;
			if(edgeDifferenceNode.edgeDifference < lowestEdgeDifference) {
				bestStartNode = edgeDifferenceNode.id;
				lowestEdgeDifference = edgeDifferenceNode.edgeDifference;
			}
		}

		//std::cout << "Graph before contraction round: " << l << std::endl << inputGraph << std::endl;
		std::vector<NodeId> independentSet;
		buildIndependentSet(inputGraph, 1000000, independentSet, initNodeSize, myBackGraph, bestStartNode);
		std::set<osmfapra::EdgeId > allEdgesToBeDeleted;
		std::vector<osmfapra::CHEdgeWithId> allEdgesToBeAdded;
		std::map<NodeId, std::vector<osmfapra::CHEdgeWithId>> edgesToBeAddedMap;
		std::vector<NodeId > nodesToContract;
		double totalEdgeDifference = 0;
		auto start = std::chrono::steady_clock::now();
		std::vector<EdgeDifferenceNode> edgeDifferenceVector;
		std::vector<NodeId> nodesToBeContracted;
		for(const auto& id : independentSet) {
			std::vector<osmfapra::CHEdgeWithId> edgesToBeAdded;
			int64_t edgeDifference = getShortCuts(inputGraph, myBackGraph, id,
														   &edgesToBeAdded);
			totalEdgeDifference += edgeDifference;
			edgeDifferenceVector.emplace_back(id, edgeDifference);
			edgesToBeAddedMap[id] = edgesToBeAdded;
		}
		auto end = std::chrono::steady_clock::now();
		std::cout << "Elapsed time in milliseconds : "
				  << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
				  << " ms" << std::endl;
		double meanEdgeDifference = totalEdgeDifference / independentSet.size();

		std::cout << meanEdgeDifference << std::endl;
		for(auto pair : edgeDifferenceVector) {
			if(pair.edgeDifference<=meanEdgeDifference) {
				for(const auto& edge : edgesToBeAddedMap[pair.id]) {
					allEdgesToBeAdded.emplace_back(edge);
				}
				nodesToBeContracted.emplace_back(pair.id);
			}
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
			GraphBuilder::removeFromVec(inputGraph.nodes, inputGraph.indexMap[id]);
		}
		std::sort(edgesToBeDeleted.begin(), edgesToBeDeleted.end());
		for(int j = edgesToBeDeleted.size() -1 ; j >= 0; --j) {
			Edge& edge = inputGraph.edges[edgesToBeDeleted[j]];
			auto it = allEdgesAdded.find(std::make_pair(edge.source, edge.target));
			if( it == allEdgesAdded.end() || edge.distance < it->second.distance) {
				auto chEdge = CHEdgeWithId{edge.source, edge.target, edge.distance, std::nullopt, std::nullopt};
				allEdgesAdded[std::make_pair(edge.source, edge.target)] = chEdge;
			}
			GraphBuilder::removeFromVec(inputGraph.edges, edgesToBeDeleted[j]);
		}
		for(auto& edge : allEdgesToBeAdded) {
			auto it = allEdgesAdded.find(std::make_pair(edge.source, edge.target));
			if( it == allEdgesAdded.end() || edge.distance < it->second.distance) {
				inputGraph.edges.emplace_back(edge);
				allEdgesAdded[std::make_pair(edge.source, edge.target)] = edge;
			}
		}
		++l;
		std::sort(inputGraph.edges.begin(), inputGraph.edges.end(), [](auto edge1, auto edge2) -> bool{
			return (edge1.source == edge2.source) ? edge1.target <= edge2.target : edge1.source < edge2.source;
		});
		start = std::chrono::steady_clock::now();
		GraphBuilder::sortNodes(inputGraph);
		end = std::chrono::steady_clock::now();
		start = std::chrono::steady_clock::now();
		GraphBuilder::buildOffset(inputGraph);
		end = std::chrono::steady_clock::now();
		std::cout << "Built offset: "
				  << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
				  << " ms" << std::endl;
		start = std::chrono::steady_clock::now();
		GraphBuilder::buildBackGraph(inputGraph, myBackGraph);
		end = std::chrono::steady_clock::now();
		std::cout << "Built backGraph in: "
				  << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
				  << " ms" << std::endl;
		start = std::chrono::steady_clock::now();
		GraphBuilder::buildOffset(myBackGraph);
		end = std::chrono::steady_clock::now();
		std::cout << "Built offest backGraph in: "
				  << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
				  << " ms with size: "  << myBackGraph.offset.size() << std::endl;
		start = std::chrono::steady_clock::now();
		GraphBuilder::buildIndexMap(inputGraph);
		end = std::chrono::steady_clock::now();
		std::cout << "Built indexmap in: "
				  << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
				  << " ms" << std::endl;
		//std::cout << "Graph after contraction: " << std::endl << inputGraph << std::endl;
	}

	visited.clear();
	visited.shrink_to_fit();
	costs.clear();
	costs.shrink_to_fit();
	inputGraph.nodes.clear();
	inputGraph.nodes.shrink_to_fit();
	inputGraph.edges.clear();
	inputGraph.edges.shrink_to_fit();
	myBackGraph.nodes.clear();
	myBackGraph.nodes.shrink_to_fit();
	myBackGraph.edges.clear();
	myBackGraph.edges.shrink_to_fit();

	GraphBuilder::sortNodes(outputGraph);
	for(auto& pair: allEdgesAdded) {
		auto& edgeWithId = pair.second;
		outputGraph.edges.emplace_back(CHEdge(edgeWithId.source, edgeWithId.target, edgeWithId.distance, std::nullopt, std::nullopt));
	}
	std::cout << "Added edges to outputgraph" << std::endl;
	std::sort(outputGraph.edges.begin(), outputGraph.edges.end(), [](auto edge1, auto edge2) -> bool{
		return (edge1.source == edge2.source) ? edge1.target <= edge2.target : edge1.source < edge2.source;
	});
	for(int i = 0; i < outputGraph.edges.size(); ++i) {
		edgeToEdgeIndexMap[std::make_pair(outputGraph.edges[i].source, outputGraph.edges[i].target)] = i;
	}
	for(auto& edge : outputGraph.edges) {
		const auto& it = allEdgesAdded.find(std::make_pair(edge.source, edge.target));
		if(it != allEdgesAdded.end()) {
			auto& children = (*it).second;
			if(children.childId2.has_value()) {
				edge.child1 = edgeToEdgeIndexMap[children.childId1.value()];
				edge.child2 = edgeToEdgeIndexMap[children.childId2.value()];
			} else {
				edge.child1 = std::nullopt;
				edge.child2 = std::nullopt;
			}
		}
	}
	allEdgesAdded.clear();
	GraphBuilder::buildOffset(outputGraph);
	GraphBuilder::buildBackGraph(outputGraph, backGraph);
	GraphBuilder::buildOffset(backGraph);
}

template<typename Graph>
void osmfapra::CHConstructor::buildIndependentSet(const Graph &graph, size_t count, std::vector<NodeId>& independentSet, std::size_t initNodeSize, const Graph& backgraph, NodeId firstNode) {
	std::vector<bool> markedNodes(initNodeSize, false);
	independentSet.clear();
	independentSet.reserve(count);
	for(size_t i = firstNode; i < graph.nodes.size(); ++i) {
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

template<typename Graph>
int64_t osmfapra::CHConstructor::getShortCuts
(const Graph& graph, const Graph& myBackGraph, osmfapra::NodeId v, std::vector<osmfapra::CHEdgeWithId>* edgesToBeAdded) {
	uint32_t edgesToBeAddedNumber = 0;
	auto& sourceEdgesBegin = myBackGraph.offset[v];
	auto& sourceEdgesEnd = myBackGraph.offset[v + 1];
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
		if(edgeFromVToTarget.source == edgeFromVToTarget.target)
			continue;
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
		auto &edgeFromVToSource = myBackGraph.edges[i];
		if(edgeFromVToSource.source == edgeFromVToSource.target) {
			break;
		}
		auto distanceFromVtoSource = edgeFromVToSource.distance;
		auto source = edgeFromVToSource.target;
		shortestDistance(graph, costs, false, maxDistanceToTarget + distanceFromVtoSource, source, v, targets, visited);
		if(distanceFromVtoSource != costs[v])
			continue;
		for(auto j = targetEdgesBegin; j < targetEdgesEnd; ++j) {
			auto& edgeFromVToTarget = graph.edges[j];
			auto& target = edgeFromVToTarget.target;
			auto distanceFromVToTarget = edgeFromVToTarget.distance;
			if(costs[target] == distanceFromVtoSource + distanceFromVToTarget && source != target){
				//did not find an other path from source to target so the shortcut is necessary
				if(source == target) continue;
				++edgesToBeAddedNumber;
				if(edgesToBeAdded != nullptr)
					edgesToBeAdded->emplace_back(CHEdgeWithId(source, target, distanceFromVtoSource + distanceFromVToTarget, std::make_pair(source, v), std::make_pair(v, target)));
			}
		}
	}
	// edge difference: #addedShortcuts - #removedEdges
	return edgesToBeAddedNumber - ((static_cast<int>(sourceEdgesEnd) - static_cast<int>(sourceEdgesBegin)) + targets.size());
}

template<typename Graph>
osmfapra::Distance
osmfapra::CHConstructor::calcDistanceLimit(const Graph &graph, const Graph &backGraph, osmfapra::NodeId source,
										   osmfapra::NodeId v, osmfapra::NodeId target) {
	return 0;
}

void osmfapra::CHConstructor::shortestDistance(const osmfapra::Graph &graph, std::vector<osmfapra::Distance> &myCosts,
											   bool ignore, osmfapra::Distance radius, osmfapra::NodeId source,
											   osmfapra::NodeId v, std::set<NodeId>& targets, std::vector<NodeId>& visitedWithoutV) {

	for(auto& nodeId : visitedWithoutV) {
		myCosts[nodeId] = MAX_DISTANCE;
	}
	visitedWithoutV.clear();
	myCosts[source] = 0;
	visitedWithoutV.emplace_back(source);
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
			if((currentEdge.target == v && ignore))
				continue;
			Distance addedCost = currentNode.cost + currentEdge.distance;
			if(myCosts[currentEdge.target] > addedCost) {
				visitedWithoutV.emplace_back(currentEdge.target);
				myCosts[currentEdge.target] = addedCost;
				queue.emplace(currentEdge.target, addedCost);
			}
		}
	}
}

template<typename Graph>
double osmfapra::CHConstructor::getAverageEdgeDifference(const Graph &graph, const Graph& myBackGraph, size_t initNodeSize, osmfapra::NodeId startNode) {
	std::vector<NodeId> independentSet;
	buildIndependentSet(inputGraph, 1000000, independentSet, initNodeSize, myBackGraph, startNode);
	std::set<osmfapra::EdgeId > allEdgesToBeDeleted;
	std::map<std::pair<NodeId, NodeId >,osmfapra::CHEdge> allEdgesToBeAdded;
	std::map<NodeId, std::vector<osmfapra::CHEdge>> edgesToBeAddedMap;
	std::vector<NodeId > nodesToContract;
	double totalEdgeDifference = 0;
	auto start = std::chrono::steady_clock::now();
	std::vector<EdgeDifferenceNode> edgeDifferenceVector;
	std::vector<NodeId> nodesToBeContracted;
	std::set<NodeId> contracted;
	for(const auto& id : independentSet) {
		std::vector<osmfapra::CHEdge> edgesToBeAdded;
		int64_t edgeDifference = getShortCuts(inputGraph, myBackGraph, id);
		totalEdgeDifference += edgeDifference;
		edgeDifferenceVector.emplace_back(id, edgeDifference);
		edgesToBeAddedMap[id] = edgesToBeAdded;
	}
	auto end = std::chrono::steady_clock::now();
	std::cout << "Elapsed time in milliseconds : "
			  << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
			  << " ms" << std::endl;
	return totalEdgeDifference / independentSet.size();
}




