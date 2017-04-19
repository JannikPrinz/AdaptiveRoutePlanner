#include "AdaptiveRoutePlanner.hpp"

AdaptiveRoutePlanner::AdaptiveRoutePlanner()
{
}

std::list<int> AdaptiveRoutePlanner::CalculateFastestRoute(const int& startNodeID, const int& targetNodeID)
{
	return list<int>();
}

bool AdaptiveRoutePlanner::AddMapData(const char* filePath)
{
	XMLError parseError = inputData.LoadFile(filePath);
	return parseError == XML_SUCCESS;
}

bool AdaptiveRoutePlanner::AddTrafficData()
{
	return true;
}

void AdaptiveRoutePlanner::ParseDocument()
{
	XMLElement* nodeElement, *wayElement, *tagElement, *tagElementHighway, *tagElementSpeedLimit, *tagElementOneway, *wayNode1, *wayNode2;
	std::unordered_map<const char*, int>::const_iterator it;
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
		tagElement = wayElement->FirstChildElement(OSM_XML_ELEMENT_TAG);
		while (tagElement != NULL)
		{
			if (tagElement->Attribute(OSM_XML_ATTRIBUTE_TAG_KEY, OSM_XML_KEY_TAG_HIGHWAY))
				tagElementHighway = tagElement;
			if (tagElement->Attribute(OSM_XML_ATTRIBUTE_TAG_KEY, OSM_XML_KEY_TAG_MAXSPEED))
				tagElementSpeedLimit = tagElement;
			if (tagElement->Attribute(OSM_XML_ATTRIBUTE_TAG_KEY, OSM_XML_KEY_TAG_ONEWAY))
				tagElementOneway = tagElement;
			tagElement = tagElement->NextSiblingElement(OSM_XML_ELEMENT_TAG);
		}

		if (tagElementHighway != NULL)
		{
			it = DEFAULT_STREETS.find(tagElementHighway->Attribute(OSM_XML_ATTRIBUTE_TAG_VALUE));
			if (it != DEFAULT_STREETS.end())
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
						mapGraph[add_edge((*it1).second, (*it2).second, mapGraph).first].costFactor = cost;
						if (!isOneway) mapGraph[add_edge((*it2).second, (*it1).second, mapGraph).first].costFactor = cost;
					}
					id = id2;
					it1 = it2;
					wayNode2 = wayNode2->NextSiblingElement(OSM_XML_ELEMENT_WAY_NODES);
				}
			}
		}
		wayElement = wayElement->NextSiblingElement(OSM_XML_ELEMENT_WAY);
	}
}

PositionType AdaptiveRoutePlanner::CalculateSquaredDistance(const VertexDescriptor& nodeID1, const VertexDescriptor& nodeID2)
{
	PositionType lat = fabs(mapGraph[nodeID1].lat - mapGraph[nodeID2].lat);
	PositionType lon = fabs(mapGraph[nodeID1].lon - mapGraph[nodeID2].lon);
	return lat * lat + lon * lon;
}

AdaptiveRoutePlanner::~AdaptiveRoutePlanner()
{
}
