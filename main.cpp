/*main.cpp*/

//
// Prof. Scott Reckinger
// University of Illinois Chicago
// CS 251: Summer 2023
// Project #05: open street maps, graphs, and Dijkstra's alg
//
// References:
// TinyXML: https://github.com/leethomason/tinyxml2
// OpenStreetMap: https://www.openstreetmap.org
// OpenStreetMap docs:
//   https://wiki.openstreetmap.org/wiki/Main_Page
//   https://wiki.openstreetmap.org/wiki/Map_Features
//   https://wiki.openstreetmap.org/wiki/Node
//   https://wiki.openstreetmap.org/wiki/Way
//   https://wiki.openstreetmap.org/wiki/Relation
//

#include <algorithm>
#include <cassert>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip> /*setprecision*/
#include <iostream>
#include <limits>
#include <map>
#include <queue>
#include <stack>
#include <string>
#include <vector>

#include "dist.h"
#include "graph.h"
#include "osm.h"
#include "tinyxml2.h"

using namespace std;
using namespace tinyxml2;

//////////////////////////////////////////////////////////////////
//
// main
//

// Function Dijkstra is declared here. It will be defined later in the program
void Dijkstra(graph<long long, double> &G, long long startV,
              map<long long, double> &distances,
              map<long long, long long> &predecessors);

int main() {
  map<long long, Coordinates>
      Nodes; // maps a Node ID to it's coordinates (lat, lon)
  vector<FootwayInfo>
      Footways; // info about each footway, in no particular order
  vector<BuildingInfo>
      Buildings; // info about each building, in no particular order
  XMLDocument xmldoc;

  cout << "** Navigating UIC open street map **" << endl;
  cout << endl;
  cout << std::setprecision(8);

  string def_filename = "map.osm";
  string filename;

  cout << "Enter map filename> ";
  getline(cin, filename);

  if (filename == "") {
    filename = def_filename;
  }

  //
  // 1. Load XML-based map file
  //
  if (!LoadOpenStreetMap(filename, xmldoc)) {
    cout << "**Error: unable to load open street map." << endl;
    cout << endl;
    return 0;
  }

  //
  // 2. Read the nodes, which are the various known positions on the map:
  //
  int nodeCount = ReadMapNodes(xmldoc, Nodes);

  //
  // 3. Read the footways, which are the walking paths:
  //
  int footwayCount = ReadFootways(xmldoc, Footways);

  //
  // 4. Read the university buildings:
  //
  int buildingCount = ReadUniversityBuildings(xmldoc, Nodes, Buildings);

  //
  // Stats
  //
  assert(nodeCount == (int)Nodes.size());
  assert(footwayCount == (int)Footways.size());
  assert(buildingCount == (int)Buildings.size());

  cout << endl;
  cout << "# of nodes: " << Nodes.size() << endl;
  cout << "# of footways: " << Footways.size() << endl;
  cout << "# of buildings: " << Buildings.size() << endl;

  graph<long long, double> G; // vertices are nodes, weights are distances

  // Step 5: Add nodes as vertices to the graph.
  for (const auto &node : Nodes) {
    G.addVertex(node.first);
  }

  // Step 6: Add edges based on footways to the graph.
  for (const auto &footway : Footways) {
    for (size_t i = 0; i < footway.Nodes.size() - 1; ++i) {
      double dist = distBetween2Points(
          Nodes[footway.Nodes[i]].Lat, Nodes[footway.Nodes[i]].Lon,
          Nodes[footway.Nodes[i + 1]].Lat, Nodes[footway.Nodes[i + 1]].Lon);

      G.addEdge(footway.Nodes[i], footway.Nodes[i + 1], dist);
      G.addEdge(footway.Nodes[i + 1], footway.Nodes[i], dist);
    }
  }

  cout << "# of vertices: " << G.NumVertices() << endl;
  cout << "# of edges: " << G.NumEdges() << endl;
  cout << endl;

  //
  // Navigation from building to building
  // 7. Input start and dest buildings
  //
  string startBuilding, destBuilding;

  // The process of searching for buildings, finding nearest nodes, running
  // Dijkstra's algorithm, and outputting distance and path is intended to be
  // implemented in this while loop

  while (startBuilding != "#") {

    cout << "Enter start (partial name or abbreviation), or #> ";
    getline(cin, startBuilding);
    if (startBuilding == "#") {
      cout << "** Done **";
      return 0;
    }
    cout << "Enter destination (partial name or abbreviation)> ";
    getline(cin, destBuilding);

    bool startFound = false, destFound = false;
    BuildingInfo startInfo, destInfo;

    // Search by abbreviation first
    for (const auto &building : Buildings) {
      if (building.Abbrev.compare(startBuilding) == 0) {
        startFound = true;
        startInfo = building;
      }
      if (building.Abbrev.compare(destBuilding) == 0) {
        destFound = true;
        destInfo = building;
      }
    }

    // If not found, then search the fullname for a partial match
    if (!startFound) {
      for (const auto &building : Buildings) {
        if (building.Fullname.find(startBuilding) != string::npos) {
          startFound = true;
          startInfo = building;
          break;
        }
      }
      if (!startFound) { // if not found at all
        cout << "Start building not found" << endl;
        continue;
      }
    }
    if (!destFound) { // run same process for destination building
      for (const auto &building : Buildings) {
        if (building.Fullname.find(destBuilding) != string::npos) {
          destFound = true;
          destInfo = building;
          break;
        }
      }
      if (!destFound) {
        cout << "Destination building not found" << endl;
        continue;
      }
    }

    // Step 8: Search the footways and find the nearest nodes to the start and
    // destination buildings
    long long startNode, destNode;
    double minStartDist = numeric_limits<double>::max(),
           minDestDist = numeric_limits<double>::max();

    for (const auto &footway : Footways) {
      for (const auto &node : footway.Nodes) {
        double startDist =
            distBetween2Points(startInfo.Coords.Lat, startInfo.Coords.Lon,
                               Nodes[node].Lat, Nodes[node].Lon);
        double destDist =
            distBetween2Points(destInfo.Coords.Lat, destInfo.Coords.Lon,
                               Nodes[node].Lat, Nodes[node].Lon);

        if (startDist < minStartDist) {
          minStartDist = startDist;
          startNode = node;
        }
        if (destDist < minDestDist) {
          minDestDist = destDist;
          destNode = node;
        }
      }
    }

    // display necessary information
    cout << "Starting point:\n"
         << " " << startInfo.Fullname << "\n (" << setprecision(8)
         << startInfo.Coords.Lat << ", " << setprecision(8)
         << startInfo.Coords.Lon << ")\n";
    cout << "Destination point:\n"
         << " " << destInfo.Fullname << "\n (" << setprecision(8)
         << destInfo.Coords.Lat << ", " << setprecision(8)
         << destInfo.Coords.Lon << ")\n";

    cout << "\nNearest start node:\n"
         << " " << startNode << "\n"
         << " (" << setprecision(8) << Nodes[startNode].Lat << ", "
         << setprecision(8) << Nodes[startNode].Lon << ")\n";

    cout << "Nearest destination node:\n"
         << " " << destNode << "\n"
         << " (" << setprecision(8) << Nodes[destNode].Lat << ", "
         << setprecision(8) << Nodes[destNode].Lon << ")\n";

    cout << "\nNavigating with Dijkstra..." << endl;

    // Step 9: Run Dijkstra’s algorithm
    map<long long, double> distances;
    map<long long, long long> predecessors;
    Dijkstra(G, startNode, distances, predecessors);

    // Step 10: Output the distance and path from start node to dest node
    if (distances[destNode] == numeric_limits<double>::max()) {
      cout << "Sorry, destination unreachable\n" << endl;
    } else {
      cout << "Distance to dest: " << setprecision(8) << distances[destNode]
           << " miles" << endl;
      stack<long long> path;
      long long v = destNode;
      while (v != startNode) {
        path.push(v);
        v = predecessors[v];
      }
      path.push(startNode);

      cout << "Path: ";
      long long prevNode = path.top();
      path.pop();
      while (!path.empty()) {
        cout << prevNode << "->";
        prevNode = path.top();
        path.pop();
      }
      cout << prevNode << "\n\n";
    }
  }

  cout << "** Done **" << endl;

  return 0;
}

// The Dijkstra function performs the Dijkstra's shortest path algorithm on a
// given graph. It initializes all vertex distances to infinity and all
// predecessors to -1. It then begins the main loop of the algorithm, updating
// distances and predecessors as it visits each vertex. The distances and
// predecessors are then used to determine and print the shortest path from the
// start to the destination.

void Dijkstra(graph<long long, double> &G, long long startV,
              map<long long, double> &distances,
              map<long long, long long> &predecessor) {

  vector<long long> visited;
  priority_queue<pair<double, long long>, vector<pair<double, long long>>,
                 greater<pair<double, long long>>>
      pq;
  map<long long, bool> isVisited;

  for (const auto &vertex : G.getVertices()) {
    distances[vertex] = numeric_limits<double>::max();
    predecessor[vertex] = -1;
    isVisited[vertex] = false;
  }

  distances[startV] = 0;
  pq.push({0, startV});

  while (!pq.empty()) {
    long long currentVertex = pq.top().second;
    pq.pop();

    if (!isVisited[currentVertex]) {
      visited.push_back(currentVertex);
      isVisited[currentVertex] = true;

      for (const auto &neighbor : G.neighbors(currentVertex)) {
        double edgeWeight;
        G.getWeight(currentVertex, neighbor, edgeWeight);

        double newDistance = distances[currentVertex] + edgeWeight;

        if (newDistance < distances[neighbor]) {
          distances[neighbor] = newDistance;
          pq.push({newDistance, neighbor});
          predecessor[neighbor] = currentVertex;
        }
      }
    }
    if (visited.size() == static_cast<size_t>(G.NumVertices())) {
      break;
    }
  }
}
