//
// Created by sokol on 27.07.19.
//

#include <osmpbf/osmfile.h>
#include <osmpbf/primitiveblockinputadaptor.h>
#include <osmpbf/include/osmpbf/filter.h>
#include <osmpbf/include/osmpbf/iway.h>
#include <osmpbf/include/osmpbf/inode.h>
#include <iostream>
#include <future>
#include <boost/iostreams/device/file_descriptor.hpp>
#include <boost/iostreams/stream.hpp>
#include <fcntl.h>
#include <iomanip>
#include "../include/GraphBuilder.h"
#include "../include/Grid.h"
#include <GeographicLib/Geodesic.hpp>

osmfapra::GraphBuilder::GraphBuilder(osmfapra::Graph &graph, std::string &file, osmfapra::GRAPH_FILETYPE &filetype, bool reorder, osmfapra::Config& config) :
	file(file), filetype(filetype), reorder(reorder), config(config){
	switch (filetype){
		case osmfapra::GRAPH_FILETYPE::PBF:
			buildPbfGraph(graph);
			break;
		case osmfapra::GRAPH_FILETYPE::FMI:
			buildFmiGraph(graph);
			break;
		case osmfapra::GRAPH_FILETYPE::CHFMI:
			return;
	}
	buildIndexMap(graph);
	buildOffset(graph);
	if(reorder) {
		reorderWithGrid(graph);
	}
	for(int i = 0; i< graph.indexMap.size(); i++)
		std::cout << i << ":" << graph.indexMap[i] << std::endl;
	std::cout << "Build offset" << std::endl;
}

auto osmfapra::GraphBuilder::buildPbfGraph(Graph& graph) -> void {
	preParseEdges(graph);
	std::cout << "Preparsed edges!" << std::endl;
	parseNodes(graph);
	std::cout << "Parsed nodes!" << std::endl;
	parseEdges(graph);
	std::cout << "Parsed edges!" << std::endl;
	osmMyIdMap.clear();
	sortEdges(graph);
	std::cout << "Sorted Edges" << std::endl;
	std::cout << "edges count: " << graph.edges.size() << std::endl;
}

void osmfapra::GraphBuilder::parseEdgesBlock(Graph &graph, osmpbf::PrimitiveBlockInputAdaptor &pbi)  {
	uint32_t highwayTagId = pbi.findString("highway");
	if (highwayTagId == 0)
		return;
	if (pbi.waysSize()) {
		for (auto way = pbi.getWayStream(); !way.isNull(); way.next()) {
			bool oneWay;
			auto onewayValue = way.valueByKey("oneway");
			bool foundHighwayKey = false;
			for(int i = 0; i < way.tagsSize(); i++) {
				if(way.key(i)== "highway")
					foundHighwayKey = true;
				if(way.key(i) == "junction")
					foundHighwayKey = true;
			}
			if(!foundHighwayKey)
				continue;
			auto highwayValue = way.valueByKey("highway");
			auto junctionValue = way.valueByKey("junction");
			if(!config.isValidHighway(highwayValue)) {
				continue;
			}
			oneWay = (onewayValue == "true" || onewayValue == "yes" || onewayValue == "1" || highwayValue == "motorway" || junctionValue == "roundabout") && highwayValue != "no";
			auto refIt(way.refBegin());
			auto nextRefIt = refIt;
			++nextRefIt;
			while (nextRefIt != way.refEnd()){
				relevantNodes.emplace(*refIt);
				relevantNodes.emplace(*nextRefIt);
				if(edgeIdSet.find(EdgeId(*refIt, *nextRefIt))==edgeIdSet.end()){
					graph.edges.emplace_back(Edge(*refIt, *nextRefIt, 0, 1));
					edgeIdSet.emplace(*refIt, *nextRefIt);
				}
				if(!oneWay && edgeIdSet.find(EdgeId(*nextRefIt, *refIt))==edgeIdSet.end()) {
					graph.edges.emplace_back(Edge(*nextRefIt, *refIt, 0, 1));
					edgeIdSet.emplace(*nextRefIt, *refIt);
				}
				++refIt;
				++nextRefIt;
			}
		}
	}
}



void osmfapra::GraphBuilder::preParseEdges(Graph& graph) {
	osmpbf::OSMFileIn inFile(file);
	if (!inFile.open()) {
		throw std::exception();
	}
	osmpbf::PrimitiveBlockInputAdaptor pbi;
	while (inFile.parseNextBlock(pbi)) {
		parseEdgesBlock(graph, pbi);
	}
}

void osmfapra::GraphBuilder::parseNodes(Graph& graph) {
	graph.nodes.reserve(relevantNodes.size());
	osmpbf::OSMFileIn inFile(file);
	if (!inFile.open()) {
		throw std::exception();
	}
	osmpbf::PrimitiveBlockInputAdaptor pbi;
	NodeId i = 0;
	std::vector<std::future<osmpbf::PrimitiveBlockInputAdaptor>> pbiFVector;
	while (inFile.parseNextBlock(pbi)) {
		if(pbi.nodesSize()) {
			for(auto node = pbi.getNodeStream(); !node.isNull(); node.next()) {
				if(relevantNodes.find(node.id()) != relevantNodes.end()) {
					graph.nodes.emplace_back(Node{ i,static_cast<Lat>(node.lond()), static_cast<Lat>(node.latd())});
					osmMyIdMap[node.id()] = i;
					++i;
				}
			}
		}

	}
	relevantNodes.clear();
	relevantNodes.rehash(0);
}

void osmfapra::GraphBuilder::parseEdges(osmfapra::Graph &graph) {
	for(auto& edge: graph.edges){
		const auto& source = graph.nodes[osmMyIdMap[edge.source]];
		const auto& target = graph.nodes[osmMyIdMap[edge.target]];
		edge = Edge(source.id, target.id, calcDistance(LatLng(source.lat, source.lng), LatLng(target.lat, target.lng)), 1);
	}
}

void osmfapra::GraphBuilder::reorderWithGrid(Graph& graph) {
	std::map<NodeId, NodeId> oldIdToNewId;
	Grid<Node> grid(std::move(graph.nodes), 180000, 360000);
	grid.buildGrid();
	graph.nodes.clear();
	NodeId i = 0;
	std::cout << "grid size: " << grid.grid.size() << std::endl;
	for(auto& pair : grid.grid) {
		auto& cellNodes = pair.second;
		for(auto& node : cellNodes) {
			oldIdToNewId[node.id] = i;
			node.id = i;
			graph.nodes.emplace_back(node);
			++i;
		}
	}
	for(auto& edge: graph.edges) {
		edge.source = oldIdToNewId[edge.source];
		edge.target = oldIdToNewId[edge.target];
	}
	sortEdges(graph);
	buildOffset(graph);
	oldIdToNewId.clear();
	grid.grid.clear();
}


void osmfapra::GraphBuilder::buildFmiGraph(Graph& graph) {
	parseFmiGraph(graph);
}

uint32_t osmfapra::GraphBuilder::findMaxNodeId() {
	osmpbf::OSMFileIn inFile(file);
	if (!inFile.open()) {
		throw std::exception();
	}
	osmpbf::PrimitiveBlockInputAdaptor pbi;
	uint32_t i = 0;
	while (inFile.parseNextBlock(pbi)) {
		if(pbi.nodesSize()) {
			for(auto node = pbi.getNodeStream(); !node.isNull(); node.next()) {
				if(node.id() > i)
					i = node.id();
			}
		}
	}
	return i;
}

namespace io = boost::iostreams;
void osmfapra::GraphBuilder::parseFmiGraph(Graph& graph) {
	uint32_t numberOfNodes{};
	uint32_t numberOfEdges{};
	int fdr = open(file.data(), O_RDONLY);
	if (fdr >= 0) {
		io::file_descriptor_source fdDevice(fdr, io::file_descriptor_flags::close_handle);
		io::stream <io::file_descriptor_source> in(fdDevice);
		if (fdDevice.is_open()) {
			std::string line;
			// ignore first 5 lines
			for(auto i = 0; i < 5; ++i) {
				std::getline(in, line);
			}
			in >> numberOfNodes;
			in >> numberOfEdges;
			Node node{};
			uint64_t osmId;
			uint32_t type;
			uint32_t i = numberOfNodes + 1;
			while ( --i > 0 && in >> node.id >> osmId >> node.lat >> node.lng >> type) {
				graph.nodes.emplace_back(node);
			}
			i = numberOfEdges + 1;
			Edge edge{};
			while (--i > 0 && in >> edge.source >> edge.target >> edge.distance >> type >> edge.maxSpeed) {
				graph.edges.emplace_back(edge);
			}
			fdDevice.close();
		}
	}
}
template <typename Graph>
void osmfapra::GraphBuilder::buildOffset(Graph& graph) {
	graph.offset.clear();
	if(graph.edges.empty()  || graph.nodes.empty())
		return;
	auto& offset = graph.offset;
	auto& nodes = graph.nodes;
	auto& edges = graph.edges;
	offset.reserve(nodes.size()+1);
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
			offset[i] = offset[j];
		}
	}
	offset[0] = 0;
	offset[offset.size()-1] = edges.size();
}

void osmfapra::GraphBuilder::buildChFmiGraph(CHGraph& graph) {
	std::cout << "building ch graph\n";
	uint32_t numberOfNodes{};
	uint32_t numberOfEdges{};
	int fdr = open(file.data(), O_RDONLY);
	if (fdr >= 0) {
		boost::iostreams::file_descriptor_source fdDevice(fdr, io::file_descriptor_flags::close_handle);
		boost::iostreams::stream <io::file_descriptor_source> in(fdDevice);
		if (fdDevice.is_open()) {
			std::string line;
			// ignore first 5 lines
			for(auto i = 0; i < 10; ++i) {
				std::getline(in, line);
			}
			in >> numberOfNodes;
			in >> numberOfEdges;

			CHNode node{};
			uint64_t osmId;
			uint32_t type;
			uint32_t i = numberOfNodes + 1;
			double lat;
			graph.nodes.reserve(numberOfNodes);
			in.precision(10);
			while ( --i > 0 && in >> node.id >> osmId >> node.lat >>  node.lng >> type >> node.level) {
				graph.nodes.emplace_back(node);
			}
			i = numberOfEdges + 1;
			graph.edges.reserve(numberOfEdges);
			CHEdge edge{};
			NodeId child1;
			NodeId child2;
			while (--i > 0 && in >> edge.source >> edge.target >> edge.distance >> type >> edge.maxSpeed >> child1 >> child2) {
				if(edge.source == edge.target)
					continue;
				if(child1 == -1){
					edge.child1 = std::nullopt;
				} else {
					edge.child1 = child1;
				}
				if(child2 == -1){
					edge.child2 = std::nullopt;
				} else {
					edge.child2 = child2;
				}
				graph.edges.emplace_back(edge);
			}
			fdDevice.close();
		}
	}
}


osmfapra::GraphBuilder::GraphBuilder(CHGraph& graph, CHGraph& backGraph, std::string& file, GRAPH_FILETYPE& filetype, bool reorder, Config& config): file(file), filetype(filetype), reorder(reorder), config(config) {
	std::cout << "loading file: " << file << "\n";
	buildChFmiGraph(graph);
	buildBackGraph(graph, backGraph);
	buildOffset(graph);
	buildOffset(backGraph);
}

osmfapra::Distance osmfapra::GraphBuilder::calcDistance(osmfapra::LatLng source, osmfapra::LatLng target) {
	const GeographicLib::Geodesic& geod = GeographicLib::Geodesic::WGS84();
	double s12;
	geod.Inverse(source.lat, source.lng, target.lat, target.lng, s12);
	return s12;
}

void osmfapra::GraphBuilder::sortEdges(Graph& graph) {
	std::sort(graph.edges.begin(), graph.edges.end(), [](auto edge1, auto edge2) -> bool{
		return (edge1.source == edge2.source) ? edge1.target <= edge2.target : edge1.source < edge2.source;
	});
}



