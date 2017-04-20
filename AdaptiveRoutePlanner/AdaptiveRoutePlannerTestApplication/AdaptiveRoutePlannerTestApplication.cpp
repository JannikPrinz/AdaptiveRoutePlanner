#include "AdaptiveRoutePlannerTestApplication.hpp"

int main()
{
	AdaptiveRoutePlanner routePlanner;
	clock_t start = clock();
	XMLError addedData = routePlanner.AddMapData("KarlsruheNW.osm");
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
	list<IdType>::iterator it = path.begin();
	while (it != path.end())
	{
		cout << (*it) << endl;
		it++;
	}

    return 0;
}