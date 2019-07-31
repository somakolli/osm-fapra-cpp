#include <iostream>
#include <Graph.h>
#include <GraphBuilder.h>
#include <Dijkstra.h>
#include <chrono>

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
	osmfapra::Graph graph;
	osmfapra::Config config{configFileName};
	osmfapra::GraphBuilder graphBuilder(graph, filename, filetype, reorder, config);
	osmfapra::Dijkstra dijkstra(graph);
	osmfapra::NodeId source, target;
	std::cout << "ready\n";
	while(true) {
		std::cin >> source;
		std::cout << "source is: " << source << '\n';
		std::cin >> target;
		std::cout << "target is: " << target << '\n';
		auto start = std::chrono::steady_clock::now();
		std::cout << dijkstra.shortestDistance(source , target)<<'\n';

		auto end = std::chrono::steady_clock::now();
		std::cout << "Elapsed time in milliseconds : "
			 << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
			 << " ms\n";
	}

	//std::cin.ignore();
  	return 0;
}
