#include "AdaptiveRoutePlanner.hpp"

AdaptiveRoutePlanner::AdaptiveRoutePlanner()
{
}

std::list<IdType> AdaptiveRoutePlanner::CalculateFastestRoute(const IdType& startNodeID, const IdType& targetNodeID)
{
	std::unordered_map<IdType, VertexDescriptor>::const_iterator start = idMap.find(startNodeID);
	std::unordered_map<IdType, VertexDescriptor>::const_iterator target = idMap.find(targetNodeID);
	if (start == idMap.end() || target == idMap.end()) return list<IdType>();
	VertexDescriptor startVertex = (*start).second;
	VertexDescriptor targetVertex = (*target).second;
	//vector<VertexDescriptor> predecessors(num_vertices(mapGraph));
	//vector<CostType> distances(num_vertices(mapGraph));

	typedef boost::unordered_map<VertexDescriptor, VertexDescriptor> pred_map;
	pred_map predecessor;
	boost::associative_property_map<pred_map> pred_pmap(predecessor);
	typedef boost::unordered_map<VertexDescriptor, PositionType> dist_map;
	dist_map distance;
	boost::associative_property_map<dist_map> dist_pmap(distance);
	boost::associative_property_map<CostMapType> cost_map(weightMap);

	list<IdType> solution;

	try
	{
		// call A*
		astar_search(
			mapGraph, startVertex,
			distance_heuristic (targetVertex, mapGraph),
			boost::weight_map(cost_map).
			predecessor_map(pred_pmap).
			distance_map(dist_pmap).
			visitor(astar_goal_visitor(targetVertex))
		);
	}
	catch (found_target ft)
	{ // found a path to the goal
		for (VertexDescriptor u = targetVertex; u != startVertex; u = predecessor[u])
			solution.push_front(mapGraph[u].nodeID);
		solution.push_front(mapGraph[startVertex].nodeID);
		//solution_length = distance[(*target).second];
		return solution;

		//list<VertexDescriptor> shortest_path;
		//for (VertexDescriptor v = (*target).second;; v = predecessors[v]) {
		//	shortest_path.push_front(v);
		//	if (predecessors[v] == v)
		//		break;
	}
	return list<IdType>();
}

XMLError AdaptiveRoutePlanner::AddMapData(const char* filePath)
{
	XMLError parseError = inputData.LoadFile(filePath);
	//inputData.SaveFile("TestXML.osm");
	ParseDocument();
	return parseError;
}

bool AdaptiveRoutePlanner::AddTrafficData()
{
	return true;
}

void AdaptiveRoutePlanner::ParseDocument()
{
	XMLElement* nodeElement, *wayElement, *tagElement, *tagElementHighway, *tagElementSpeedLimit, *tagElementOneway, *tagElementVehicle, *wayNode1, *wayNode2;
	std::unordered_map<string, int>::const_iterator it;
	std::unordered_map<IdType, VertexDescriptor>::const_iterator it1, it2, mapEnd;
	VertexDescriptor vertexD;
	int speedLimit;
	IdType id, id2;
	CostType cost;
	bool isOneway;
	// add all nodes
	nodeElement = inputData.FirstChildElement(OSM_XML_ELEMENT_OSM)->FirstChildElement(OSM_XML_ELEMENT_NODE);
	while (nodeElement != NULL)
	{
		vertexD = add_vertex(mapGraph);
		id = strtoull(nodeElement->Attribute(OSM_XML_ATTRIBUTE_NODE_ID), nullptr, 10);
		idMap.emplace(id, vertexD);
		mapGraph[vertexD] = VertexProperty(id, atof(nodeElement->Attribute(OSM_XML_ATTRIBUTE_NODE_LON)), atof(nodeElement->Attribute(OSM_XML_ATTRIBUTE_NODE_LAT)));
		nodeElement = nodeElement->NextSiblingElement(OSM_XML_ELEMENT_NODE);
	}
	// add all ways
	wayElement = inputData.FirstChildElement(OSM_XML_ELEMENT_OSM)->FirstChildElement(OSM_XML_ELEMENT_WAY);
	while (wayElement != NULL)
	{
		// add all edges of the way, if the way is a suitable road
		tagElementHighway = NULL;
		tagElementSpeedLimit = NULL;
		tagElementOneway = NULL;
		tagElementVehicle = NULL;
		tagElement = wayElement->FirstChildElement(OSM_XML_ELEMENT_TAG);
		while (tagElement != NULL)
		{
			if (tagElement->Attribute(OSM_XML_ATTRIBUTE_TAG_KEY, OSM_XML_KEY_TAG_HIGHWAY))
				tagElementHighway = tagElement;
			if (tagElement->Attribute(OSM_XML_ATTRIBUTE_TAG_KEY, OSM_XML_KEY_TAG_MAXSPEED))
				tagElementSpeedLimit = tagElement;
			if (tagElement->Attribute(OSM_XML_ATTRIBUTE_TAG_KEY, OSM_XML_KEY_TAG_ONEWAY))
				tagElementOneway = tagElement;
			if (tagElement->Attribute(OSM_XML_ATTRIBUTE_TAG_KEY, OSM_XML_KEY_TAG_VEHICLE))
				tagElementVehicle = tagElement;
			tagElement = tagElement->NextSiblingElement(OSM_XML_ELEMENT_TAG);
		}

		if (tagElementHighway != NULL)
		{
			it = DEFAULT_STREETS.find(tagElementHighway->Attribute(OSM_XML_ATTRIBUTE_TAG_VALUE));
			if (it != DEFAULT_STREETS.end() && ((tagElementVehicle == NULL) || !(tagElementVehicle->Attribute(OSM_XML_ATTRIBUTE_TAG_VALUE, OSM_XML_VALUE_VEHICLE_NO))))
			{
				// -> way is suitable, check speed limits:
				if (tagElementSpeedLimit != NULL)
				{
					speedLimit = atoi(tagElementSpeedLimit->Attribute(OSM_XML_ATTRIBUTE_TAG_VALUE));
				}
				else
				{
					speedLimit = (*it).second;
				}
				// todo: check oneway, extract node ids, check if nodes exist, add edges
				// check if the way is a oneway:
				if (tagElementOneway != NULL)
				{
					isOneway = (tagElementOneway->Attribute(OSM_XML_ATTRIBUTE_TAG_VALUE, OSM_XML_VALUE_ONEWAY_YES)) ? true : false;
				}
				else
				{
					isOneway = false;
				}
				// go through all saved nodes of the way:
				wayNode1 = wayElement->FirstChildElement(OSM_XML_ELEMENT_WAY_NODES);
				if (wayNode1 != NULL)
				{
					wayNode2 = wayNode1->NextSiblingElement(OSM_XML_ELEMENT_WAY_NODES);
					id = strtoull(wayNode1->Attribute(OSM_XML_ATTRIBUTE_WAY_NODES_ID), nullptr, 10);
					it1 = idMap.find(id);
				}
				else
				{
					wayNode2 = NULL;
				}
				mapEnd = idMap.end();
				while (wayNode2 != NULL)
				{
					id2 = strtoull(wayNode2->Attribute(OSM_XML_ATTRIBUTE_WAY_NODES_ID), nullptr, 10);
					// check, if nodes exist in graph:
					it2 = idMap.find(id2);
					if (it1 != mapEnd && it2 != mapEnd)
					{
						cost = CalculateSquaredDistance((*it1).second, (*it2).second) / speedLimit;
						//mapGraph[add_edge((*it1).second, (*it2).second, mapGraph).first].costFactor = cost;
						//if (!isOneway) mapGraph[add_edge((*it2).second, (*it1).second, mapGraph).first].costFactor = cost;
						weightMap.emplace(add_edge((*it1).second, (*it2).second, mapGraph).first, cost);
						if (!isOneway) weightMap.emplace(add_edge((*it2).second, (*it1).second, mapGraph).first, cost);
					}
					id = id2;
					it1 = it2;
					wayNode2 = wayNode2->NextSiblingElement(OSM_XML_ELEMENT_WAY_NODES);
				}
			}
		}
		wayElement = wayElement->NextSiblingElement(OSM_XML_ELEMENT_WAY);
	}
	inputData.Clear();
}

CostType AdaptiveRoutePlanner::CalculateSquaredDistance(const VertexDescriptor& nodeID1, const VertexDescriptor& nodeID2)
{
	CostType lat = mapGraph[nodeID1].lat - mapGraph[nodeID2].lat;
	CostType lon = mapGraph[nodeID1].lon - mapGraph[nodeID2].lon;
	return lat * lat + lon * lon;
}

AdaptiveRoutePlanner::~AdaptiveRoutePlanner()
{
}
