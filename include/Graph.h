#ifndef _Graph_hpp_
#define _Graph_hpp_

#include <vector>
#include <cstdint>
#include <map>
#include <iostream>

namespace osmfapra
{
using NodeId = int64_t;
using Lat = float;
using Lng = float;
using Distance = uint64_t ;
using Speed = int32_t;
using MySize = uint32_t;
using Level = uint32_t ;
class Node
{
public:
	NodeId id;
    Lat lat;
    Lng lng;
	friend std::ostream &operator<<(std::ostream & Str, const Node & node) {
		Str << node.lat << ' ' << node.lng;
		return Str;
	}
};

class CHNode : public Node {
public:
	Level level;
	friend std::ostream &operator<<(std::ostream & Str, const CHNode & node) {
		Str << (Node) node << ' ' << node.level;
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
	friend std::ostream &operator<<(std::ostream & Str, const Edge & edge) {
		Str << edge.source << ' ' << edge.target << ' ' << edge.distance << ' ' << edge.maxSpeed;
		return Str;
	}
};

class CHEdge : public Edge {
public:
	NodeId child1;
	NodeId child2;
	friend std::ostream &operator<<(std::ostream & Str, const CHEdge & edge) {
		Str << (Edge)edge << ' ' << edge.child1 << ' ' << edge.child2;
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
    Graph() = default;
    ~Graph() = default;
	friend std::ostream &operator<<(std::ostream & Str, const Graph & graph) {
		for(auto node: graph.nodes) {
			Str << node;
		}
		for(auto edge: graph.edges) {
			Str << edge;
		}
		for(auto i = 0; i < graph.offset.size(); ++i) {
			Str << i << ":" << graph.offset[i] << std::endl;
		}
		Str << "nodes: " << graph.nodes.size() << std:: endl;
		Str << "edges: " << graph.edges.size() << std::endl;
		Str << "offset: " << graph.offset.size() << std::endl;
		return Str;
	}
};

class CHGraph : public Graph {
public:
	std::vector<CHNode> nodes;
	std::vector<CHEdge> edges;
};





} // namespace osmfapra


#endif