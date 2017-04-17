#pragma once
#ifndef ADAPTIVEROUTEPLANNER_H
#define ADAPTIVEROUTEPLANNER_H

#include <list>

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
	bool AddMapData();
	bool AddTrafficData();

private:

};

#endif // !ADAPTIVEROUTEPLANNER_H
