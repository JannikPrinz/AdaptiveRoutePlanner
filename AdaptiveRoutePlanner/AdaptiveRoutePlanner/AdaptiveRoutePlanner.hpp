#pragma once
#ifndef ADAPTIVEROUTEPLANNER_HPP
#define ADAPTIVEROUTEPLANNER_HPP

#include <list>
#include <boost\graph\astar_search.hpp>
#include <boost\graph\adjacency_list.hpp>
#include "tinyxml2.h"
#include "AdaptiveRoutePlannerConstants.hpp"

using namespace std;
using namespace tinyxml2;
using namespace boost;

typedef unsigned long long IdType;
typedef float PositionType;
typedef float CostType;

/*
 * This struct defines which attributes are saved per vertex.
 */
struct VertexProperty
{
	// NodeID from the parsed XML-Document (OSM-Data)
	IdType nodeID;
	PositionType lon;
	PositionType lat;

	VertexProperty(IdType i = 0, PositionType lo = 0.0, PositionType la = 0.0) : nodeID(i), lon(lo), lat(la) {}
};

/*
 * This struct defines which attributes are saved per edge.
 */
struct EdgeProperty
{
	// cost factor which represents the time needed to travel along the edge using factors like
	// distance, speed limit, ...
	//CostType costFactor;
};

typedef adjacency_list<vecS, vecS, directedS, VertexProperty, no_property> GraphType;
//typedef adjacency_list<vecS, listS, directedS, VertexProperty, EdgeProperty> GraphType;
typedef graph_traits<GraphType>::vertex_descriptor VertexDescriptor;
typedef graph_traits<GraphType>::edge_descriptor EdgeDescriptor;
typedef boost::unordered_map<EdgeDescriptor, CostType> CostMapType;

static CostType CalculateSquaredDistance(const VertexDescriptor& nodeID1, const VertexDescriptor& nodeID2, const GraphType& mapGraph)
{
	CostType lat = mapGraph[nodeID1].lat - mapGraph[nodeID2].lat;
	CostType lon = mapGraph[nodeID1].lon - mapGraph[nodeID2].lon;
	return lat * lat + lon * lon;
}

// euclidean distance heuristic
class distance_heuristic : public astar_heuristic<GraphType, CostType>
{
public:
	distance_heuristic(VertexDescriptor goal, GraphType mGraph) : m_goal(goal), mapGraph(mGraph) {}
	CostType operator()(VertexDescriptor u)
	{
		return CalculateSquaredDistance(u, m_goal, mapGraph);
	}
private:
	Vertex m_goal;
	GraphType mapGraph;
};

struct found_target {}; // exception for termination

// visitor that terminates when we find the goal
class astar_goal_visitor : public boost::default_astar_visitor
{
public:
	astar_goal_visitor(VertexDescriptor goal) : m_goal(goal) {}
	void examine_vertex(VertexDescriptor u, const GraphType& g)
	{
		if (u == m_goal)
			throw found_target();
	}
private:
	VertexDescriptor m_goal;
};

class AdaptiveRoutePlanner
{
public:
	AdaptiveRoutePlanner();
	~AdaptiveRoutePlanner();
	/*
	 * This method uses the current saved map-data to calculate a fastest route between the given
	 * start- and target-Node.
	 *
	 * Parameters:
	 * const IdType& startNodeID	: ID of the node from where the route starts
	 * const IdType& targetNodeID	: ID of the node where the route ends
	 *
	 * Returns:
	 * A list of node-id's, where the first id is the given start id and the last id is the given target id.
	 * For each pair of id's (a, b) where b is the successor of a, exists an (directed) edge between node a to node b.
	 */
	std::list<IdType> CalculateFastestRoute(const IdType& startNodeID, const IdType& targetNodeID);
	/*
	 * This method adds new map-data from a XML-Document which will be used for the route calculation.
	 *
	 * Parameters:
	 * const char* filePath : path of the XML-Document which will be parsed
	 *
	 * Returns:
	 * True, if the parsing of the document doesn't throw any errors. False otherwise.
	 */
	bool AddMapData(const char* filePath);
	bool AddTrafficData();

private:
	XMLDocument inputData;
	GraphType mapGraph;
	std::unordered_map<IdType, VertexDescriptor> idMap;
	CostMapType weightMap;

	CostType CalculateSquaredDistance(const VertexDescriptor& nodeID1, const VertexDescriptor& nodeID2);
	void ParseDocument();
};

#endif // !ADAPTIVEROUTEPLANNER_HPP
