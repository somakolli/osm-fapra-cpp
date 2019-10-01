#ifndef _Graph_hpp_
#define _Graph_hpp_

#include <vector>
#include <cstdint>
#include <map>
#include <iostream>
#include <cmath>
#include <iomanip>
#include "list"
#include "stack"

namespace osmfapra
{
using NodeId = uint32_t;
using Lat = float;
using Lng = float;
using Distance = uint32_t ;
using Speed = int16_t;
using MySize = uint32_t;
using Level = uint32_t ;
using EdgeId = std::pair<NodeId, NodeId>;
class Node
{
public:
	NodeId id;
    Lat lat;
    Lng lng;
	friend std::ostream &operator<<(std::ostream & Str, const Node & node) {
		Str << std::setprecision(10) << "id: "<< node.id <<" lat: " << node.lat << " lon: " << node.lng;
		return Str;
	}
};

class CHNode : public Node {
public:
	Level level;
	friend std::ostream &operator<<(std::ostream & Str, const CHNode & node) {
		Str << (Node) node << ' ' << " level:" << node.level;
		return Str;
	}
};

class Edge
{
public:
    NodeId source;
    NodeId target;
    Distance distance;

	Edge() = default;

	Edge(NodeId source, NodeId target, Distance distance) : source(source), target(target),
																			distance(distance){}

	friend std::ostream &operator<<(std::ostream & Str, const Edge & edge) {
		Str << "source: " << edge.source << ' ' << "target: " << edge.target << " distance: " << edge.distance;
		return Str;
	}



	friend bool operator==(const Edge &rhs, const Edge &lhs) {
		return rhs.source == lhs.source &&
				rhs.target == lhs.target &&
				rhs.distance == lhs.distance;
	}

	bool operator!=(const Edge &rhs) const {
		return !(rhs == *this);
	}
};

class LatLng {
public:
	Lat lat;
	Lng lng;
	LatLng(Lat lat, Lng lng) : lat(lat), lng(lng) {}
	friend std::ostream &operator<<(std::ostream & Str, const LatLng & latLng) {
		Str << latLng.lat << ";" << latLng.lng;
		return Str;
	}
};

class CHEdge : public Edge {
public:
	std::optional<NodeId> child1;
	std::optional<NodeId> child2;

	CHEdge() : Edge() {}

	CHEdge(NodeId source, NodeId target, Distance distance, std::optional<NodeId> child1, std::optional<NodeId> child2) : Edge(source,
																												 target,
																												 distance),
																											child1(child1),
																											child2(child2) {}

	friend bool operator==(const CHEdge &rhs, const CHEdge &lhs) {
		return static_cast<const osmfapra::Edge &>(rhs) == static_cast<const osmfapra::Edge &>(lhs);
	}

	friend bool operator!=(const CHEdge &rhs, const CHEdge &lhs) {
		return !(rhs == lhs);
	}

	friend std::ostream &operator<<(std::ostream & Str, const CHEdge & edge) {
		Str << (Edge)edge << " child1: " << edge.child1.value_or(-1) << " child2: " << edge.child2.value_or(-1);
		return Str;
	}
};
class CHEdgeWithId: public Edge {
public:
	CHEdgeWithId() : Edge() {}
	std::optional<EdgeId> childId1;
	std::optional<EdgeId> childId2;

	CHEdgeWithId(NodeId source, NodeId target, Distance distance, std::optional<EdgeId> childId1,
				 std::optional<EdgeId> childId2);
};
class Graph
{
private:
public:
	std::vector<Node> nodes;
	std::vector<Edge> edges;
	std::vector<MySize> offset;
	std::vector<NodeId> indexMap;
    Graph() = default;
    ~Graph() = default;

	friend std::ostream &operator<<(std::ostream & Str, const Graph & graph) {
		auto i = 0;
		for(auto node: graph.nodes) {
			Str << "id: " << i << node << '\n';
			i++;
		}
		for(auto edge: graph.edges) {
			Str << edge << '\n';
		}
		for(i = 0; i < graph.offset.size(); ++i) {
			Str << i << ":" << graph.offset[i] << std::endl;
		}
		Str << "indexMap:" << std::endl;
		for(i = 0; i < graph.indexMap.size(); ++i) {
			Str << i << ":" << graph.indexMap[i] << std::endl;
		}
		Str << "nodes: " << graph.nodes.size() << std:: endl;
		Str << "edges: " << graph.edges.size() << std::endl;
		Str << "offset: " << graph.offset.size() << std::endl;
		return Str;
	}
	template <typename GraphT>
	static NodeId getClosestNode(GraphT& graph, osmfapra::LatLng latLng) {
		double smallestDistance = std::numeric_limits<double>::max();
		auto nearestNode = graph.nodes[0];
		for(auto node : graph.nodes) {
			double distance = std::pow(latLng.lat - node.lat, 2) + std::pow(latLng.lng - node.lng, 2);
			if(distance < smallestDistance){
				smallestDistance = distance;
				nearestNode = node;
			}
		}
		return nearestNode.id;
	};
};

class CHGraph : public Graph {
public:
	std::vector<CHNode> nodes;
	std::vector<CHEdge> edges;
	friend std::ostream &operator<<(std::ostream & Str, const CHGraph & graph) {
		auto i = 0;
		for(auto& node: graph.nodes) {
			Str << "id: " << i << node << '\n';
			i++;
		}
		for(auto& edge: graph.edges) {
			Str << edge << '\n';
		}
		for(i = 0; i < graph.offset.size(); ++i) {
			Str << i << ":" << graph.offset[i] << std::endl;
		}
		Str << "nodes: " << graph.nodes.size() << std:: endl;
		Str << "edges: " << graph.edges.size() << std::endl;
		Str << "offset: " << graph.offset.size() << std::endl;
		return Str;
	}
	std::vector<NodeId> getPathFromShortcut(CHEdge shortcut) {
		std::vector<NodeId> path;
		path.emplace_back(shortcut.source);
		if(!shortcut.child2.has_value()) {
			//std::cout << "is not shortcut" << std::endl;
			path.emplace_back(shortcut.source);
			path.emplace_back(shortcut.target);
			return path;
		}
		//std::cout << "found shortcut" << std::endl;
		std::stack<uint32_t> edgesStack;

		edgesStack.push(shortcut.child2.value());
		edgesStack.push(shortcut.child1.value());

		while (!edgesStack.empty()) {
			auto& edgeIdx = edgesStack.top();
			edgesStack.pop();
			const auto& edge = edges[edgeIdx];
			//std::cout << "edge from stack: " << edge << std::endl;
			//std::cout << "source: " << nodes[edge.source] << std::endl;
			//std::cout << "target: " << nodes[edge.target] << std::endl;
			if(edge.child1.has_value()) {
				//std::cout << "is shortcut" << std::endl;

				edgesStack.push(edge.child2.value());
				edgesStack.push(edge.child1.value());
			} else {
				//std::cout << "is not shortcut" << std::endl;
				path.emplace_back(edge.target);
			}
		}
		return path;
	}
};
} // namespace osmfapra


#endif