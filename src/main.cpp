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
#include <fstream>
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
	if(splitMessage[0]== "p") {
		osmfapra::Lat lat1 = std::stod(splitMessage[1]);
		osmfapra::Lat lat2 = std::stod(splitMessage[3]);
		osmfapra::Lng lng1 = std::stod(splitMessage[2]);
		osmfapra::Lng lng2 = std::stod(splitMessage[4]);
		const auto& path = dijkstra.shortestPath(osmfapra::LatLng{lat1, lng1}, osmfapra::LatLng{lat2, lng2});
		std::string response;
		for(const auto& pathLatLng: path) {
			response+= std::to_string(pathLatLng.lat) + ',' + std::to_string(pathLatLng.lng) + ',';
		}
		std::cout << "response: " << response << std::endl;
		return response;
	}
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
	auto bufferSize = 1000000;
	char buffer[bufferSize];

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

int main(int argc, char *argv[]) {
	std::string filename;
	std::string configFileName;
	osmfapra::GRAPH_FILETYPE filetype = osmfapra::GRAPH_FILETYPE::PBF;
	bool reorder = true;
	uint rounds = 0;
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
		} else if (option == "-rounds") {
			rounds = std::stoi(argv[++i]);
			i+=1;
		} else {
			std::cout << "Unrecognized commands!" << std::endl;
			return 1;
		}
	}
	osmfapra::Config config{configFileName};
	osmfapra::Config* configPtr = &config;
	if(filetype == osmfapra::GRAPH_FILETYPE::FMI) {
		configPtr = nullptr;
	}
	osmfapra::Graph graph;
	osmfapra::CHGraph chGraph;
	osmfapra::GraphBuilder graphBuilder(graph, filename, filetype, reorder, configPtr);
	osmfapra::CHGraph backGraph;
	osmfapra::CHConstructor chConstructor(graph, chGraph, backGraph, rounds);
	osmfapra::CHDijkstra chDijkstra(chGraph, backGraph);
	startServerLoop(chDijkstra);
	return 0;
}


