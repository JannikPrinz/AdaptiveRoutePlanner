#include "AdaptiveRoutePlannerTestApplication.hpp"

#define FILENAME "KarlsruheNW.osm"

int main()
{
	AdaptiveRoutePlanner routePlanner;
	clock_t start = clock();
	XMLError addedData = routePlanner.AddMapData(FILENAME);
	cout << "Time needed for adding and parsing map-data: " << (double)(clock() - start) / CLOCKS_PER_SEC << " seconds." << endl;
	string s = "";
	s.append(EnumStrings[addedData]);
	//s.append((addedData) ? "Map is successfully loaded." : "Couldn't load map data.");

	cout << s << endl;

	cout << "Number of vertices in graph: " << num_vertices(routePlanner.mapGraph) << endl;
	cout << "Number of edges in graph: " << num_edges(routePlanner.mapGraph) << endl;

	start = clock();
	//list<IdType> path = routePlanner.CalculateFastestRoute(15357760, 15357760);
	//list<IdType> path = routePlanner.CalculateFastestRoute(15357760, 31152150);
	list<IdType> path = routePlanner.CalculateFastestRoute(26940051, 55474925);
	cout << "Time needed for calculating fastest route: " << (double)(clock() - start) / CLOCKS_PER_SEC << " seconds." << endl;
	cout << "Number of nodes in shortest path: " << path.size() << endl;

	// write route on map, download renderer here: http://maperitive.net
	TinyXMLDocument ModifiedDocument;
	ModifiedDocument.LoadFile(FILENAME);
	XMLElement* el = ModifiedDocument.FirstChildElement(OSM_XML_ELEMENT_OSM);
	XMLElement* newElement = ModifiedDocument.NewElement("way");
	newElement->SetAttribute("id", 9999999999);
	newElement->SetAttribute("visisble", true);
	newElement->SetAttribute("version", 1);
	newElement->SetAttribute("user", "AdaptiveRoutePlanner");
	list<IdType>::iterator it = path.begin();
	for (size_t i = 0; i < path.size(); i++)
	{
		XMLElement* nd = ModifiedDocument.NewElement("nd");
		nd->SetAttribute("ref", (int64_t)(*it));
		newElement->InsertEndChild(nd);
		it++;
	}
	XMLElement* tag = ModifiedDocument.NewElement("tag");
	tag->SetAttribute("k", "highway");
	tag->SetAttribute("v", "motorway");
	newElement->InsertEndChild(tag);
	el->InsertEndChild(newElement);
	ModifiedDocument.SaveFile("RouteMap.osm");

	// print all node-ids:
	it = path.begin();
	while (it != path.end())
	{
		cout << (*it) << endl;
		it++;
	}

    return 0;
}