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
	std::vector<OsmId> collectedOsmNodes;
	void buildPbfGraph(Graph& graph);
	void preParseEdges(Graph& graph);
	void parseNodes(Graph& graph);
	static void parseNodesBlock(Graph& graph,osmpbf::PrimitiveBlockInputAdaptor& pbi, std::unordered_set<uint32_t >& relevantNodes);
	uint32_t findMaxNodeId();
	void reorderWithGrid(Graph& graph);
	void buildFmiGraph(Graph& graph);
	void parseFmiGraph(Graph& graph);
	template <typename Graph>
	void buildOffset(Graph& graph);
public:

	std::unordered_set<OsmId> relevantNodes;
	GraphBuilder(Graph& graph, std::string& file, GRAPH_FILETYPE& filetype, bool reorder, Config& config);
	GraphBuilder(CHGraph& graph, CHGraph& backGraph, std::string& file, GRAPH_FILETYPE& filetype, bool reorder, Config& config);
	~GraphBuilder() = default;

	void buildChFmiGraph(CHGraph& graph);

	void buildBackGraph(CHGraph& graph, CHGraph& backGraph);
};

}



#endif //OSM_FAPRA_CPP_GRAPHBUILDER_H
