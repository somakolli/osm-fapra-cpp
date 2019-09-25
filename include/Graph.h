#ifndef _Graph_hpp_
#define _Graph_hpp_

#include <vector>
#include <cstdint>
#include <map>
#include <iostream>
#include <cmath>

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
		Str << "id: "<< node.id <<" lat: " << node.lat << " lon: " << node.lng;
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
    Speed maxSpeed;
    bool deleted = false;

	Edge() = default;

	Edge(NodeId source, NodeId target, Distance distance, Speed maxSpeed) : source(source), target(target),
																			distance(distance), maxSpeed(maxSpeed) {}

	friend std::ostream &operator<<(std::ostream & Str, const Edge & edge) {
		Str << "source: " << edge.source << ' ' << "target: " << edge.target << " distance: " << edge.distance << " maxSpeed: " << edge.maxSpeed;
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

	CHEdge(NodeId source, NodeId target, Distance distance, Speed maxSpeed, std::optional<NodeId> child1, std::optional<NodeId> child2) : Edge(source,
																												 target,
																												 distance,
																												 maxSpeed),
																											child1(child1),
																											child2(child2) {}

	friend bool operator==(const CHEdge &rhs, const CHEdge &lhs) {
		return static_cast<const osmfapra::Edge &>(rhs) == static_cast<const osmfapra::Edge &>(lhs);
	}

	friend bool operator!=(const CHEdge &rhs, const CHEdge &lhs) {
		return !(rhs == lhs);
	}

	friend std::ostream &operator<<(std::ostream & Str, const CHEdge & edge) {
		Str << (Edge)edge << " child1: " << edge.child1.value_or(0) << " child2: " << edge.child2.value_or(0);
		return Str;
	}
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
};
} // namespace osmfapra


#endif