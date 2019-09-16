#include <Graph.h>
#include <vector>
#include <cmath>
#include <GeographicLib/Geodesic.hpp>
#include <iomanip>

using namespace GeographicLib;


osmfapra::NodeId osmfapra::CHGraph::getClosestNode(osmfapra::Lat lat, osmfapra::Lng lng) {
	const Geodesic& geod = Geodesic::WGS84();
	double smallestDistance = std::numeric_limits<double>::max();
	CHNode nearestNode = nodes[0];
	for(auto node : nodes) {
		double distance = std::pow(lat - node.lat, 2) + std::pow(lng - node.lng, 2);
		if(distance < smallestDistance){
			smallestDistance = distance;
			nearestNode = node;
		}
	}
	return nearestNode.id;
}
