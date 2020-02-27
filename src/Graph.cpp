#include <utility>

#include "../include/Graph.h"

osmfapra::CHEdgeWithId::CHEdgeWithId(osmfapra::NodeId source, osmfapra::NodeId target, osmfapra::Distance distance,
									 std::optional<osmfapra::EdgeId> childId1,
									 std::optional<osmfapra::EdgeId> childId2) : Edge(source, target, distance),
																						childId1(std::move(childId1)),
																						childId2(std::move(childId2)) {}
