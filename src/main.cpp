#include <iostream>
#include <chrono>
#include <unistd.h>
#include <stdio.h>
#include <sys/socket.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <string.h>
#include <boost/algorithm/string.hpp>
#include <arpa/inet.h>
#include "../include/Graph.h"
#include "../include/GraphBuilder.h"
#include "../include/CHDijkstra.h"
#include "../include/CHConstructor.h"
#include "cstdlib"

#define PORT 4301
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-noreturn"

template <typename Dijkstra>
std::string processMessage(std::string message, Dijkstra& dijkstra) {
	std::cout << "recieved message: " << message << '\n';
	std::vector<std::string> splitMessage;
	boost::split(splitMessage, message, [](char c){return c == ',';});
	size_t sourcesSize = std::stoi(splitMessage[0]);
	size_t targetsSize = std::stoi(splitMessage[1]);
	std::vector<osmfapra::LatLng> sources;
	std::vector<osmfapra::LatLng> targets;
	for(auto i = 2; i < sourcesSize+2; i+=2) {
		osmfapra::Lat lat2 = std::stod(splitMessage[i]);
		osmfapra::Lng lng2 = std::stod(splitMessage[i+1]);
		sources.emplace_back(osmfapra::LatLng{lat2,lng2});
	}
	for(auto i = sourcesSize+2; i < targetsSize+sourcesSize+2; i+=2) {
		osmfapra::Lat lat2 = std::stod(splitMessage[i]);
		osmfapra::Lng lng2 = std::stod(splitMessage[i+1]);
		targets.emplace_back(osmfapra::LatLng{lat2,lng2});
	}
	auto distances = dijkstra.multiSourceMultiTarget(sources, targets);
	std::string distance;
	std::cout << sources.size() << std::endl;
	for(auto i = 0; i < distances.size(); i++) {
		distance += std::to_string(distances[i]);
		if(i != distances.size()-1) {
			distance += ',';
		}
	}
	std::cout << "test" << std::endl;
	std::cout << "distances: " << distance << std::endl;
	return distance;
}

// source: https://www.geeksforgeeks.org/socket-programming-in-cc-handling-multiple-clients-on-server-without-multi-threading/
template <typename DijkstraT>
void startServerLoop(DijkstraT &dijkstra) {

	int opt = true;
	int master_socket , addrlen , new_socket , client_socket[30] ,
			max_clients = 30 , activity, i , valread , sd;
	int max_sd;
	struct sockaddr_in address;
	auto bufferSize = 10000;
	char buffer[bufferSize];  //data buffer of 1K

	//set of socket descriptors
	fd_set readfds;

	//initialise all client_socket[] to 0 so not checked
	for (i = 0; i < max_clients; i++)
	{
		client_socket[i] = 0;
	}

	//create a master socket
	if( (master_socket = socket(AF_INET , SOCK_STREAM , 0)) == 0)
	{
		perror("socket failed");
		exit(EXIT_FAILURE);
	}

	//set master socket to allow multiple connections ,
	//this is just a good habit, it will work without this
	if( setsockopt(master_socket, SOL_SOCKET, SO_REUSEADDR, (char *)&opt,
				   sizeof(opt)) < 0 )
	{
		perror("setsockopt");
		exit(EXIT_FAILURE);
	}

	//type of socket created
	address.sin_family = AF_INET;
	address.sin_addr.s_addr = INADDR_ANY;
	address.sin_port = htons( PORT );

	//bind the socket to localhost port 8888
	if (bind(master_socket, (struct sockaddr *)&address, sizeof(address))<0)
	{
		perror("bind failed");
		exit(EXIT_FAILURE);
	}
	printf("Listener on port %d \n", PORT);

	//try to specify maximum of 3 pending connections for the master socket
	if (listen(master_socket, 3) < 0)
	{
		perror("listen");
		exit(EXIT_FAILURE);
	}

	//accept the incoming connection
	addrlen = sizeof(address);
	puts("Waiting for connections ...");

	while(true)
	{
		//clear the socket set
		FD_ZERO(&readfds);

		//add master socket to set
		FD_SET(master_socket, &readfds);
		max_sd = master_socket;

		//add child sockets to set
		for ( i = 0 ; i < max_clients ; i++)
		{
			//socket descriptor
			sd = client_socket[i];

			//if valid socket descriptor then add to read list
			if(sd > 0)
				FD_SET( sd , &readfds);

			//highest file descriptor number, need it for the select function
			if(sd > max_sd)
				max_sd = sd;
		}

		//wait for an activity on one of the sockets , timeout is NULL ,
		//so wait indefinitely
		activity = select( max_sd + 1 , &readfds , NULL , NULL , NULL);

		if ((activity < 0) && (errno!=EINTR))
		{
			printf("select error");
		}

		//If something happened on the master socket ,
		//then its an incoming connection
		if (FD_ISSET(master_socket, &readfds))
		{
			if ((new_socket = accept(master_socket,
									 (struct sockaddr *)&address, (socklen_t*)&addrlen))<0)
			{
				perror("accept");
				exit(EXIT_FAILURE);
			}

			puts("Welcome message sent successfully");

			//add new socket to array of sockets
			for (i = 0; i < max_clients; i++)
			{
				//if position is empty
				if( client_socket[i] == 0 )
				{
					client_socket[i] = new_socket;
					printf("Adding to list of sockets as %d\n" , i);

					break;
				}
			}
		}

		//else its some IO operation on some other socket
		for (i = 0; i < max_clients; i++)
		{
			sd = client_socket[i];

			if (FD_ISSET( sd , &readfds))
			{
				//Check if it was for closing , and also read the
				//incoming message
				if ((valread = read( sd , buffer, bufferSize-1)) == 0)
				{
					//Somebody disconnected , get his details and print
					getpeername(sd , (struct sockaddr*)&address , \
                        (socklen_t*)&addrlen);
					printf("Host disconnected , ip %s , port %d \n" ,
						   inet_ntoa(address.sin_addr) , ntohs(address.sin_port));

					//Close the socket and mark as 0 in list for reuse
					close( sd );
					client_socket[i] = 0;
				}
				else
				{
					//set the string terminating NULL byte on the end
					//of the data read
					buffer[valread] = '\0';
					std::string newMessage{buffer};
					auto response = processMessage(newMessage, dijkstra);
					const char* messageCharPointer = response.c_str();
					send(sd, messageCharPointer, response.size(), 0);
				}
			}
		}
	}
}
#pragma clang diagnostic pop

template <typename DijkstraT, typename CHDijkstra>
void startCliLoop(DijkstraT &dijkstra, CHDijkstra &chDijkstra, size_t graphSize, bool ch) {
	for(int i = 0; i < graphSize && false; ++i) {
		for(int j = 0; j < graphSize; ++j) {
			auto start = std::chrono::steady_clock::now();
			int64_t dijkstraD = dijkstra.shortestDistance(i, j);
			auto end = std::chrono::steady_clock::now();
			auto start2 = std::chrono::steady_clock::now();
			int64_t chDijkstraD = chDijkstra.shortestDistance(i, j);
			auto end2 = std::chrono::steady_clock::now();
			if(dijkstraD == chDijkstraD) {
				std::cout << "distances didnt match for " << i << " " << j << " distance: " << dijkstraD << " chdistance: " << chDijkstraD <<  std::endl;
				std::cout << "Dijkstra took : "
						  << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
						  << " ms" << std::endl;
				std::cout << "chDijkstra : "
						  << std::chrono::duration_cast<std::chrono::milliseconds>(end2 - start2).count()
						  << " ms" << std::endl;
			}
		}
	}
	while(true) {
		std::cout << "enter source" << '\n';
		osmfapra::NodeId source;
		std::cin >> source;
		std::cout << "enter target" << '\n';
		osmfapra::NodeId target;
		std::cin >> target;
		auto start = std::chrono::steady_clock::now();
		std::cout << "Distance: " << dijkstra.shortestDistance(source, target) << '\n';
		auto end = std::chrono::steady_clock::now();
		std::cout << "Dijkstra took : "
				  << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count()
				  << " ms" << std::endl;
		if(ch) {
			auto start2 = std::chrono::steady_clock::now();
			std::cout << "ch Distance: " << chDijkstra.shortestDistance(source, target) << '\n';
			auto end2 = std::chrono::steady_clock::now();
			std::cout << "chDijkstra : "
					  << std::chrono::duration_cast<std::chrono::microseconds>(end2 - start2).count()
					  << " ms" << std::endl;
		}
	}
}


int main(int argc, char *argv[]) {
	std::string filename;
	std::string configFileName;
	osmfapra::GRAPH_FILETYPE filetype;
	bool print = false;
	bool reorder = false;
	bool cli  = false;
	bool ch = true;
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
		} else if (option == "-cli") {
			cli = true;
			i+=1;
		} else if (option == "-print") {
			print = true;
			i+=1;
		}else if (option == "-noCh") {
			ch = false;
			i+=1;
		}
		else {
			std::cout << "Unrecognized commands!" << std::endl;
			return 1;
		}
	}

	osmfapra::Config config{configFileName};
	osmfapra::Graph graph;
	osmfapra::GraphBuilder graphBuilder(graph, filename, filetype, reorder, config);
	osmfapra::CHGraph chGraph;
	osmfapra::CHGraph backGraph;
	if(ch)
		osmfapra::CHConstructor chConstructor(graph, chGraph, backGraph);
	std::cout << "ch constructed!" << std::endl;
	osmfapra::Graph graph1;
	osmfapra::GraphBuilder graphBuilder1(graph1, filename, filetype, reorder, config);
	osmfapra::Dijkstra dijkstra(graph1);
	if(print)
		std::cout << graph << std::endl;
	osmfapra::CHDijkstra chDijkstra(chGraph, backGraph);
	if(cli)
		startCliLoop(dijkstra, chDijkstra, graph.nodes.size(), ch);
	startServerLoop(dijkstra);
	return 0;
}


