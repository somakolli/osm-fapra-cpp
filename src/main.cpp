#include <iostream>
#include <Graph.h>
#include <GraphBuilder.h>
#include <Dijkstra.h>
#include <chrono>
#include <CHDijkstra.h>

template <typename DijkstraT>
void startLoop(DijkstraT& dijkstra) {
	while(true) {
		osmfapra::NodeId source, target;
		std::cin >> source;
		std::cout << "source is: " << source << '\n';
		std::cin >> target;
		std::cout << "target is: " << target << '\n';
		auto start = std::chrono::steady_clock::now();
		auto shortestDistance =  dijkstra.shortestDistance(source , target);

		auto end = std::chrono::steady_clock::now();
		std::cout << "Elapsed time in milliseconds : "
				  << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
				  << " ms\n";
		std::cout << "Shortest distance: " << shortestDistance << '\n';
	}
}

int main(int argc, char *argv[]) {
	std::string filename;
	std::string configFileName;
	osmfapra::GRAPH_FILETYPE filetype;
	bool reorder = false;
	uint16_t i = 1;
	while(i < argc) {
		std::string option = argv[i];
		if(option == "-t") {
			std::string filetypeString = argv[i+1];
			if(filetypeString == "pbf") {
				filetype = osmfapra::GRAPH_FILETYPE::PBF;
			} else if (filetypeString == "fmi") {
				filetype = osmfapra::GRAPH_FILETYPE::FMI;
			} else if (filetypeString == "chfmi") {
				filetype = osmfapra::GRAPH_FILETYPE::CHFMI;
			} else {
				std::cout << "Not supported filetype!" << std::endl;
				return 1;
			}
			i += 2;
		} else if (option == "-f") {
			filename = argv[i + 1];
			i += 2;
		} else if (option == "-c") {
			configFileName = argv[i + 1];
			i += 2;
		} else if (option == "-gridReorder") {
			i+=1;
			reorder = true;
		}
		else {
			std::cout << "Unrecognized commands!" << std::endl;
			return 1;
		}
	}

	osmfapra::Config config{configFileName};
	if(filetype == osmfapra::GRAPH_FILETYPE::CHFMI) {
		osmfapra::CHGraph chGraph;
		osmfapra::CHGraph backGraph;
		osmfapra::GraphBuilder graphBuilder(chGraph, backGraph, filename, filetype, reorder, config);
		std::cout << "ready\n";
		osmfapra::CHDijkstra chDijkstra(chGraph, backGraph);
		startLoop(chDijkstra);
	}

	osmfapra::Graph graph;
	osmfapra::GraphBuilder graphBuilder(graph, filename, filetype, reorder, config);
	osmfapra::Dijkstra dijkstra(graph);
	std::cout << "ready\n";
	startLoop(dijkstra);

	//std::cin.ignore();
  	return 0;
}


