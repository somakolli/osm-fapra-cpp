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
#include "CHConstructor.h"

namespace osmfapra{

enum GRAPH_FILETYPE{
	FMI,
	PBF
};

using OsmId = uint32_t ;

struct OsmNode {
	OsmId id;
	Lat lat;
	Lng lng;
};
class GraphBuilder {
private:
	Config* config;
	std::string& file;
	GRAPH_FILETYPE& filetype;
	bool reorder = false;
	std::set<EdgeId> edgeIdSet;
	std::map<OsmId, NodeId> osmMyIdMap;
	void buildPbfGraph(Graph& graph);
	void preParseEdges(Graph& graph);
	void parseNodes(Graph& graph);
	void parseEdges(Graph& graph);
	void parseEdgesBlock(Graph &graph, osmpbf::PrimitiveBlockInputAdaptor &pbi);
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
	static void buildOffset(Graph& graph){
		graph.offset.clear();
		if(graph.edges.empty()  || graph.nodes.empty())
			return;
		auto& offset = graph.offset;
		auto& nodes = graph.nodes;
		auto& edges = graph.edges;
		offset.reserve(nodes[nodes.size()-1].id+1);
		while(offset.size() <= nodes[nodes.size()-1].id+1) {
			offset.emplace_back(0);
		}
		offset[nodes[nodes.size() - 1].id] = edges.size();
		for(int i = edges.size()-1; i >= 0; --i ){
			offset[edges[i].source] = i;
		}
		for(int i = edges[0].source + 1; i < offset.size()-2; ++i) {

			if(offset[i]==0) {
				size_t j = i+1;
				while(offset[j]==0){
					++j;
				}
				size_t offsetToSet = offset[j];
				--j;
				size_t firstNullPosition = i;
				while(j >= firstNullPosition) {
					offset[j] = offsetToSet;
					--j;
					++i;
				}
			}
		}
		offset[0] = 0;
		offset[offset.size()-1] = edges.size();
	}

	std::unordered_set<OsmId> relevantNodes;
	template <typename Graph>
	GraphBuilder(Graph& graph, std::string& file, GRAPH_FILETYPE& filetype, bool reorder, Config* config = nullptr) :
			file(file), filetype(filetype), reorder(reorder), config(config){
		switch (filetype){
			case osmfapra::GRAPH_FILETYPE::PBF:
				buildPbfGraph(graph);
				break;
			case osmfapra::GRAPH_FILETYPE::FMI:
				buildFmiGraph(graph);
				break;
		}
		buildIndexMap(graph);
		buildOffset(graph);
		if(reorder) {
			reorderWithGrid(graph);
		}
	}
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
		graph.indexMap.clear();
		graph.indexMap.resize(graph.nodes[graph.nodes.size()-1].id + 1);
		for(int i = 0; i < graph.nodes.size(); ++i)
			graph.indexMap[graph.nodes[i].id] = i;
	}
	template <typename T, typename IndexType>
	static void removeFromVec(std::vector<T>& vec, IndexType index) {
		if(vec.empty())
			return;
		vec[index] = vec.back();
		vec.pop_back();
	}

};

}



#endif //OSM_FAPRA_CPP_GRAPHBUILDER_H
