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
	Config& config;
	Graph& graph;
	std::string& file;
	GRAPH_FILETYPE& filetype;
	bool reorder = false;
	std::vector<OsmId> collectedOsmNodes;
	void buildPbfGraph();
	void preParseEdges();
	void parseNodes();
	static void parseNodesBlock(osmpbf::PrimitiveBlockInputAdaptor& pbi, std::unordered_set<uint32_t >& relevantNodes);
	uint32_t findMaxNodeId();
	void reorderWithGrid();
	void buildFmiGraph();
	void parseFmiGraph();
	void buildOffset();
public:

	std::unordered_set<OsmId> relevantNodes;
	GraphBuilder(Graph& graph, std::string& file, GRAPH_FILETYPE& filetype, bool reorder, Config& config);
	~GraphBuilder() = default;

};

}



#endif //OSM_FAPRA_CPP_GRAPHBUILDER_H
