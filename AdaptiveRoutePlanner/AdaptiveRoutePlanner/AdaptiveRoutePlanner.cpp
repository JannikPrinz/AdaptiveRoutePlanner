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
	// add all nodes
	XMLElement* nodeElement = inputData.FirstChildElement(OSM_XML_ELEMENT_OSM)->FirstChildElement(OSM_XML_ELEMENT_NODE);
	while (nodeElement != NULL)
	{
		idMap.emplace(atoi(nodeElement->Attribute(OSM_XML_ATTRIBUTE_NODE_ID)), add_vertex(mapGraph));
		nodeElement = nodeElement->NextSiblingElement(OSM_XML_ELEMENT_NODE);
	}
	// add all ways
	XMLElement* wayElement = inputData.FirstChildElement(OSM_XML_ELEMENT_OSM)->FirstChildElement(OSM_XML_ELEMENT_WAY);
	while (wayElement != NULL)
	{
		// add all edges of the way, if the way is a suitable road
		XMLElement* tagElement = wayElement->FirstChildElement(OSM_XML_ELEMENT_TAG);
		while (tagElement != NULL)
		{
			// todo...
			tagElement = tagElement->NextSiblingElement(OSM_XML_ELEMENT_TAG);
		}
		wayElement = wayElement->NextSiblingElement(OSM_XML_ELEMENT_WAY);
	}
}

AdaptiveRoutePlanner::~AdaptiveRoutePlanner()
{
}
