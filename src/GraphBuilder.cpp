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

osmfapra::GraphBuilder::GraphBuilder(osmfapra::Graph &graph, std::string &file, osmfapra::GRAPH_FILETYPE &filetype, bool reorder, Config& config) :
		graph(graph), file(file), filetype(filetype), reorder(reorder), config(config){
	switch (filetype){
		case osmfapra::GRAPH_FILETYPE::PBF:
			buildPbfGraph();
			break;
		case osmfapra::GRAPH_FILETYPE::FMI:
			buildFmiGraph();
			break;
	}
	buildOffset();
}

void osmfapra::GraphBuilder::buildPbfGraph() {

	preParseEdges();
	std::cout << "Preparsed edges!" << std::endl;
	parseNodes();
	std::cout << "Parsed nodes!" << std::endl;
}

void osmfapra::GraphBuilder::parseNodesBlock(osmpbf::PrimitiveBlockInputAdaptor &pbi,
											 std::unordered_set<uint32_t >& relevantNodes) {
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

void osmfapra::GraphBuilder::preParseEdges() {
	osmpbf::OSMFileIn inFile(file);
	if (!inFile.open()) {
		throw std::exception();
	}
	std::vector<std::future<void>> futureVec;
	osmpbf::PrimitiveBlockInputAdaptor pbi;
	while (inFile.parseNextBlock(pbi)) {
		parseNodesBlock(pbi, relevantNodes);
		//futureVec.emplace_back(std::async(std::launch::deferred, parseNodesBlock, std::ref(pbi), std::ref(relevantNodes)));
	}
	for(auto& future: futureVec) {
		future.get();
	}
}

void osmfapra::GraphBuilder::parseNodes() {
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
		reorderWithGrid();
	relevantNodes.clear();
	relevantNodes.rehash(0);
}

void osmfapra::GraphBuilder::reorderWithGrid() {
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

void osmfapra::GraphBuilder::buildFmiGraph() {
	parseFmiGraph();
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
void osmfapra::GraphBuilder::parseFmiGraph() {
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

void osmfapra::GraphBuilder::buildOffset() {
	auto& offset = graph.offset;
	auto& nodes = graph.nodes;
	auto& edges = graph.edges;

	offset.reserve(nodes.size()+1);
	while(offset.size() < nodes.size()) {
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



