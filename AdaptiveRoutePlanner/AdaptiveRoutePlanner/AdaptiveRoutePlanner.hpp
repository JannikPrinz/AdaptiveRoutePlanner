#pragma once
#ifndef ADAPTIVEROUTEPLANNER_HPP
#define ADAPTIVEROUTEPLANNER_HPP

#include <list>
#include <boost\graph\astar_search.hpp>
#include "tinyxml2.h"

using namespace std;
using namespace tinyxml2;

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
	 * const int& startNodeID	: ID of the node from where the route starts
	 * const int& targetNodeID	: ID of the node where the route ends
	 *
	 * Returns:
	 * A list of node-id's, where the first id is the given start id and the last id is the given target id.
	 * For each pair of id's (a, b) where b is the successor of a, exists an (directed) edge between node a to node b.
	 */
	std::list<int> CalculateFastestRoute(const int& startNodeID, const int& targetNodeID);
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

};

#endif // !ADAPTIVEROUTEPLANNER_HPP
