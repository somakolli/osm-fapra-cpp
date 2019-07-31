#ifndef _Graph_hpp_
#define _Graph_hpp_

#include <vector>
#include <cstdint>
#include <map>
#include <iostream>

namespace osmfapra
{
using NodeId = uint32_t;
using Lat = float;
using Lng = float;
using Distance = float;
using Speed = uint16_t;
using MySize = uint32_t;
class Node
{
public:
	NodeId id;
    Lat lat;
    Lng lng;
	friend std::ostream &operator<<(std::ostream & Str, const Node & node) {
		Str << node.lat << ' ' << node.lng << '\n';
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
		Str << edge.source << ' ' << edge.target << ' ' << edge.distance << ' ' << edge.maxSpeed << '\n';
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





} // namespace osmfapra


#endif