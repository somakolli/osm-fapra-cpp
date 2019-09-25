//
// Created by sokol on 27.07.19.
//

#ifndef OSM_FAPRA_CPP_GRAPHBUILDER_H
#define OSM_FAPRA_CPP_GRAPHBUILDER_H

#include "Graph.h"
#include "Config.h"
#include <string>
#include <unordered_set>
#include <map>
#include <vector>
#include <osmpbf/include/osmpbf/primitiveblockinputadaptor.h>
#include <algorithm>
#include "map"
#include "set"

namespace osmfapra{

enum GRAPH_FILETYPE{
	FMI,
	PBF,
	CHFMI
};

using OsmId = uint32_t ;

struct OsmNode {
	OsmId id;
	Lat lat;
	Lng lng;
};
class GraphBuilder {
private:
	Config& config;
	std::string& file;
	GRAPH_FILETYPE& filetype;
	bool reorder = false;
	std::set<NodeId> nodeIdSet;
	std::set<EdgeId> edgeIdSet;
	std::map<OsmId, NodeId> osmMyIdMap;
	size_t edgesSize = 0;
	void buildPbfGraph(Graph& graph);
	void preParseEdges(Graph& graph);
	void parseNodes(Graph& graph);
	void parseEdges(Graph& graph);
	void parseEdgesBlock(Graph &graph, osmpbf::PrimitiveBlockInputAdaptor &pbi);
	uint32_t findMaxNodeId();
	void reorderWithGrid(Graph& graph);
	void buildFmiGraph(Graph& graph);
	void parseFmiGraph(Graph& graph);
	Distance calcDistance(LatLng source, LatLng target);
	void sortEdges(Graph& graph);
public:
	template <typename Graph>
	static void sortNodes(Graph& graph){
		std::sort(graph.nodes.begin(), graph.nodes.end(), [](auto edge1, auto edge2) -> bool{
			return edge1.id < edge2.id;
		});
	}
	template <typename Graph>
	static void buildOffset(Graph& graph);

	std::unordered_set<OsmId> relevantNodes;
	GraphBuilder(Graph& graph, std::string& file, GRAPH_FILETYPE& filetype, bool reorder, Config& config);
	GraphBuilder(CHGraph& graph, CHGraph& backGraph, std::string& file, GRAPH_FILETYPE& filetype, bool reorder, Config& config);
	~GraphBuilder() = default;

	void buildChFmiGraph(CHGraph& graph);

	template <typename Graph1, typename Graph2>
	static void buildBackGraph(const Graph1& graph, Graph2& backGraph) {
		backGraph.edges.clear();
		backGraph.nodes.clear();
		backGraph.offset.clear();
		backGraph.nodes.reserve(graph.nodes.size());
		for(auto node : graph.nodes) {
			backGraph.nodes.emplace_back(node);
		}
		backGraph.edges.reserve(graph.edges.size());
		for(auto edge: graph.edges) {
			auto source = edge.source;
			edge.source = edge.target;
			edge.target = source;
			backGraph.edges.emplace_back(edge);
		}

		std::sort(backGraph.edges.begin(), backGraph.edges.end(), [](auto edge1, auto edge2) -> bool{
			return (edge1.source == edge2.source) ? edge1.target <= edge2.target : edge1.source < edge2.source;
		});
	}
	template <typename Graph>
	static void buildIndexMap(Graph& graph) {
		graph.indexMap.resize(graph.nodes[graph.nodes.size()-1].id + 1);
		for(int i = 0; i < graph.nodes.size(); ++i)
			graph.indexMap[graph.nodes[i].id] = i;
	}
};

}



#endif //OSM_FAPRA_CPP_GRAPHBUILDER_H
