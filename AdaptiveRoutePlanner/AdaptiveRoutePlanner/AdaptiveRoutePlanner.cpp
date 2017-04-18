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

AdaptiveRoutePlanner::~AdaptiveRoutePlanner()
{
}