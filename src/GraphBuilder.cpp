//
// Created by sokol on 27.07.19.
//

#include "GraphBuilder.h"
#include <osmpbf/osmfile.h>
#include <osmpbf/primitiveblockinputadaptor.h>
#include <osmpbf/include/osmpbf/filter.h>
#include <osmpbf/include/osmpbf/iway.h>
#include <osmpbf/include/osmpbf/inode.h>
#include <Grid.h>
#include <iostream>
#include <future>
#include <boost/iostreams/device/file_descriptor.hpp>
#include <boost/iostreams/stream.hpp>
#include <fcntl.h>
#include <iomanip>

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
	buildOffset(graph);
}

auto osmfapra::GraphBuilder::buildPbfGraph(Graph& graph) -> void {
	preParseEdges(graph);
	std::cout << "Preparsed edges!" << std::endl;
	parseNodes(graph);
	std::cout << "Parsed nodes!" << std::endl;
}

void osmfapra::GraphBuilder::parseNodesBlock(Graph& graph, osmpbf::PrimitiveBlockInputAdaptor &pbi,
											 std::unordered_set<uint32_t> &relevantNodes)  {
	osmpbf::AndTagFilter filter({new osmpbf::KeyOnlyTagFilter("highway")});
	filter.assignInputAdaptor(&pbi);
	if (filter.rebuildCache()) {
		if (pbi.waysSize()) {
			for (auto way = pbi.getWayStream(); !way.isNull(); way.next()) {
				if (filter.matches(way)) {
					auto refIt(way.refBegin());
					auto nextRefIt = refIt;
					++nextRefIt;
					while (nextRefIt != way.refEnd()){
						relevantNodes.emplace(*refIt);
						relevantNodes.emplace(*nextRefIt);
						++refIt;
						++nextRefIt;
					}
				}
			}
		}
	}
}

void osmfapra::GraphBuilder::preParseEdges(Graph& graph) {
	osmpbf::OSMFileIn inFile(file);
	if (!inFile.open()) {
		throw std::exception();
	}
	std::vector<std::future<void>> futureVec;
	osmpbf::PrimitiveBlockInputAdaptor pbi;
	while (inFile.parseNextBlock(pbi)) {
		parseNodesBlock(graph, pbi, relevantNodes);
		//futureVec.emplace_back(std::async(std::launch::deferred, parseNodesBlock, std::ref(pbi), std::ref(relevantNodes)));
	}
	for(auto& future: futureVec) {
		future.get();
	}
}

void osmfapra::GraphBuilder::parseNodes(Graph& graph) {
	graph.nodes.reserve(relevantNodes.size());
	osmpbf::OSMFileIn inFile(file);
	if (!inFile.open()) {
		throw std::exception();
	}
	osmpbf::PrimitiveBlockInputAdaptor pbi;
	uint32_t i = 0;
	std::vector<std::future<osmpbf::PrimitiveBlockInputAdaptor>> pbiFVector;
	while (inFile.parseNextBlock(pbi)) {
		if(pbi.nodesSize()) {
			for(auto node = pbi.getNodeStream(); !node.isNull(); node.next()) {
				if(relevantNodes.find(node.id()) != relevantNodes.end()) {
					graph.nodes.emplace_back(Node{ i,static_cast<Lat>(node.lond()), static_cast<Lat>(node.latd())});
				}
			}
		}
		++i;
	}
	if(reorder)
		reorderWithGrid(graph);
	relevantNodes.clear();
	relevantNodes.rehash(0);
}

void osmfapra::GraphBuilder::reorderWithGrid(Graph& graph) {
	Grid<Node> grid(std::move(graph.nodes), 18000, 36000);
	grid.buildGrid();
	uint32_t i = 0;
	for(const auto& pair : grid.grid) {
		const auto& cellNodes = pair.second;
		for(const auto& node : cellNodes) {
			graph.nodes.emplace_back(std::move(node));
			++i;
		}
	}
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
		boost::iostreams::file_descriptor_source fdDevice(fdr, io::file_descriptor_flags::close_handle);
		boost::iostreams::stream <io::file_descriptor_source> in(fdDevice);
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
			while ( --i > 0 && in >> node.id >> osmId >> node.lat >>  node.lng >> type) {
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
	auto& offset = graph.offset;
	auto& nodes = graph.nodes;
	auto& edges = graph.edges;
	offset.reserve(nodes.size()+1);
	while(offset.size() < nodes.size()+1) {
		offset.emplace_back(0);
	}
	for(size_t i = edges.size()-1; i > 0; --i ){
		offset[edges[i].source] = i;
	}
	for(size_t i = 1; i < nodes.size()-1; ++i) {
		if(offset[i]==0) {
			size_t j = i+1;
			while(offset[j]==0){
				++j;
			}
			offset[i] = offset[j];
		}
	}
	offset[nodes.size()] = edges.size();
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
			while ( --i > 0 && in  >> node.id >> osmId >> node.lat >>  node.lng >> type >> node.level) {
				graph.nodes.emplace_back(node);
			}
			i = numberOfEdges + 1;
			graph.edges.reserve(numberOfEdges);
			CHEdge edge{};
			while (--i > 0 && in >> edge.source >> edge.target >> edge.distance >> type >> edge.maxSpeed >> edge.child1 >> edge.child2) {
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

void osmfapra::GraphBuilder::buildBackGraph(CHGraph& graph, CHGraph& backGraph) {

	backGraph.nodes.reserve(graph.nodes.size());
	for(auto node : graph.nodes) {
		backGraph.nodes.emplace_back(node);
	}
	backGraph.edges.reserve(graph.edges.size());
	for(const auto& edge: graph.edges) {
		backGraph.edges.emplace_back(CHEdge{edge.target, edge.source, edge.distance, edge.maxSpeed, edge.child1, edge.child2});
	}
	std::sort(backGraph.edges.begin(), backGraph.edges.end(), [](CHEdge edge1, CHEdge edge2) -> bool{
		return (edge1.source == edge2.source) ? edge1.target <= edge2.target : edge1.source < edge2.source;
	});
}



