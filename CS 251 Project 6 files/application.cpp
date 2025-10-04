#include "application.h"

#include <iostream>
#include <limits>
#include <map>
#include <queue> // priority_queue
#include <set>
#include <stack>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "dist.h"
#include "graph.h"

#include "json.hpp"
using json = nlohmann::json;

using namespace std;

double INF = numeric_limits<double>::max();



// ● "buildings": a list of (most) UIC buildings as vertices
// ● "waypoints": a list of non-building vertices
// ● "footways": a list of OSM ways, as described above

// [Note: Created by LLM, following the project instructions, only debugging done manually]
void buildGraph(istream &input,
                graph<long long, double> &g,
                vector<BuildingInfo> &buildings,
                unordered_map<long long, Coordinates> &coords) {
  // 1) Read-In JSOn
  json j;
  input >> j;

  // 2) buildings: add each building as a vertex(graph), only store into 'buildings' vector.
  for (auto const &jb : j["buildings"]) {
    long long id   = jb["id"].get<long long>();
    double lat     = jb["lat"].get<double>();
    double lon     = jb["lon"].get<double>();
    string abbr    = jb["abbr"].get<string>();
    string name    = jb["name"].get<string>();

    g.addVertex(id);
    buildings.emplace_back(id, Coordinates{lat, lon}, name, abbr);
  }

  // 3) waypoints: add each waypoint was a vertex(graph), store into coordinates
  for (auto const &jw : j["waypoints"]) {
    long long id   = jw["id"].get<long long>();
    double lat     = jw["lat"].get<double>();
    double lon     = jw["lon"].get<double>();

    g.addVertex(id);
    coords[id] = Coordinates{lat, lon};
  }

  // 4) footways: an array of IDs. Connect consecutive pairs.
  for (auto const &way : j["footways"]) {
    for (size_t i = 1; i < way.size(); ++i) {
      long long u = way[i - 1].get<long long>();
      long long v = way[i    ].get<long long>();

      // if (!coords.contains(u)) {
      //   return 0;
      // }
      // else if (!coords.contains(v)) {
      //   return 0;
      // }

      // Check if both points actually exist.
      if (coords.contains(u) && coords.contains(v)) { // manually added during debugging process
        double d = distBetween2Points(coords[u], coords[v]);
        g.addEdge(u, v, d);
        g.addEdge(v, u, d);
      }
    }
  }

  // 5) Link buildings to a waypoint, within 0.036 distance.
  const double R = 0.036;
  for (auto const &b : buildings) {
    for (auto const &wp_pair : coords) {
      long long wp = wp_pair.first;
      double d = distBetween2Points(b.location, wp_pair.second);
      if (d <= R) {
        g.addEdge(b.id, wp, d);
        g.addEdge(wp, b.id, d);
      }
    }
  }
}


BuildingInfo getBuildingInfo(const vector<BuildingInfo> &buildings,
                             const string &query) {
  for (const BuildingInfo &building : buildings) {
    if (building.abbr == query) {
      return building;
    } else if (building.name.find(query) != string::npos) {
      return building;
    }
  }

  //else {}
  BuildingInfo fail;
  fail.id = -1;
  return fail;
}

BuildingInfo getClosestBuilding(const vector<BuildingInfo> &buildings,
                                Coordinates c) {
  double minDestDist = INF;
  BuildingInfo ret = buildings.at(0);
  for (const BuildingInfo &building : buildings) {
    double dist = distBetween2Points(building.location, c);
    if (dist < minDestDist) {
      minDestDist = dist;
      ret = building;
    }
  }
  return ret;
}




// [TODO: Implement Dijkstra]
class prioritize {
public:
  bool operator()(const pair<long long, double>& p1,
      const pair<long long, double>& p2) const {
      
      return p1.second > p2.second;
  }
};


// [TODO: Implement Dijkstra]
// Note: Returning a path: vertices from 'start' to 'target'
//  Unreachable -> return empty
//  if (start == target) -> return a vector, only with 'start'
//  Consider ignoreVertices
vector<long long> dijkstra(const graph<long long, double> &G, long long start,
                           long long target,
                           const set<long long> &ignoreNodes) {



  //Referring lecture worksheet, define seen, distance, predecessor
  unordered_set<long long> seen;
  unordered_map<long long, double> distance;
  unordered_map<long long, long long> predecessor;

  // Define priority_queue : worklist
  // Store both the vertices, and the best known distances to those vertices.
  // long long : store IDs
  priority_queue<pair<long long, double>, 
    vector<pair<long long, double>>,prioritize> worklist;

  if (start == target) {
    return vector<long long>{start};
  }

  // Using getVertice of graph.h, call the vector of vertex(s)
  // G: vector of vertices

  // Set default status
  for (auto vertex: G.getVertices()) {
    distance[vertex] = INF; // Initiate as all INF

  }
    distance[start] = 0; // starts at 'start'-> distance 0
    worklist.push({start, 0});


  while (!worklist.empty()) {

    //bring the first(top) element
    pair<long long, double> top = worklist.top();


    worklist.pop(); // remove 'top' from worklist (ie.. marking as used?)

    long long curr = top.first;
    double distCurr = top.second;

    seen.insert(curr);

    if (curr == target) {
      // return vector<long long>{start};
      break;
    }

    // Now, for curr, find neighbors
    for (auto neighbor : G.neighbors(curr)) {
      if (ignoreNodes.contains(neighbor) && neighbor != start && neighbor != target) {
        continue;  // skip the ignoring node
      } 
      

      //else

      double weight;
      bool gotWeight = G.getWeight(curr, neighbor, weight);

      if (!gotWeight) {
        cout << "[Debug]: Weight Not Accessible" << endl;
        break;
      }
      // now we have weight for curr, neighbor

      double distCurr_add_weight = distCurr + weight;

      if (distCurr_add_weight < distance[neighbor]) {
        //update distance[neighbor]
        distance[neighbor] = distCurr_add_weight;

        predecessor[neighbor] = curr;
        worklist.push({neighbor, distCurr_add_weight});
      }
    }
  }



  // Unreachable -> return empty vec
  if (distance[target] == INF) {
    cout << "[Debug]: Unreachable" << endl;
    return{};
  }

  //else (we can get the path, with start, target on each ends)

  vector<long long> path;
  long long p = target;

  while (p != start) {
    path.push_back(p);
    p = predecessor[p];
  }

  path.push_back(start);


  vector<long long> path_reversed(path.size());
  for (int i = 0 ; i < path.size(); i++) {
    path_reversed[i] = path[path.size() - 1 - i];
  }

  return path_reversed;
  // return vector<long long>{};
}




  // while (there are elements in the heap) {
  
  //   if (already explored node) {
  //     continue; (skip)
  //   }
  //   else {
  //     add the first node to explored set

  //     for (each connected node) {
  //       if (alreadt explored) {
  //         continue (skip)
  //       }
  //       if (distance of the node + distance < distance of connected node) {
  //         update the distance of connected noded;
  //         add updated distance to the connected node
  //       }
  //     }
  //   }
  // }
  // return distance of target(Destination)


double pathLength(const graph<long long, double> &G,
                  const vector<long long> &path) {
  double length = 0.0;
  double weight;
  for (size_t i = 0; i + 1 < path.size(); i++) {
    bool res = G.getWeight(path.at(i), path.at(i + 1), weight);
    if (!res) {
      return -1;
    }
    length += weight;
  }
  return length;
}

void outputPath(const vector<long long> &path) {
  for (size_t i = 0; i < path.size(); i++) {
    cout << path.at(i);
    if (i != path.size() - 1) {
      cout << "->";
    }
  }
  cout << endl;
}

// Honestly this function is just a holdover from an old version of the project
void application(const vector<BuildingInfo> &buildings,
                 const graph<long long, double> &G) {
  string person1Building, person2Building;

  set<long long> buildingNodes;
  for (const auto &building : buildings) {
    buildingNodes.insert(building.id);
  }

  cout << endl;
  cout << "Enter person 1's building (partial name or abbreviation), or #> ";
  getline(cin, person1Building);

  while (person1Building != "#") {
    cout << "Enter person 2's building (partial name or abbreviation)> ";
    getline(cin, person2Building);

    // Look up buildings by query
    BuildingInfo p1 = getBuildingInfo(buildings, person1Building);
    BuildingInfo p2 = getBuildingInfo(buildings, person2Building);
    Coordinates P1Coords, P2Coords;
    string P1Name, P2Name;

    if (p1.id == -1) {
      cout << "Person 1's building not found" << endl;
    } else if (p2.id == -1) {
      cout << "Person 2's building not found" << endl;
    } else {
      cout << endl;
      cout << "Person 1's point:" << endl;
      cout << " " << p1.name << endl;
      cout << " " << p1.id << endl;
      cout << " (" << p1.location.lat << ", " << p1.location.lon << ")" << endl;
      cout << "Person 2's point:" << endl;
      cout << " " << p2.name << endl;
      cout << " " << p2.id << endl;
      cout << " (" << p2.location.lon << ", " << p2.location.lon << ")" << endl;

      Coordinates centerCoords = centerBetween2Points(p1.location, p2.location);
      BuildingInfo dest = getClosestBuilding(buildings, centerCoords);

      cout << "Destination Building:" << endl;
      cout << " " << dest.name << endl;
      cout << " " << dest.id << endl;
      cout << " (" << dest.location.lat << ", " << dest.location.lon << ")"
           << endl;

      vector<long long> P1Path = dijkstra(G, p1.id, dest.id, buildingNodes);
      vector<long long> P2Path = dijkstra(G, p2.id, dest.id, buildingNodes);

      // This should NEVER happen with how the graph is built
      if (P1Path.empty() || P2Path.empty()) {
        cout << endl;
        cout << "At least one person was unable to reach the destination "
                "building. Is an edge missing?"
             << endl;
        cout << endl;
      } else {
        cout << endl;
        cout << "Person 1's distance to dest: " << pathLength(G, P1Path);
        cout << " miles" << endl;
        cout << "Path: ";
        outputPath(P1Path);
        cout << endl;
        cout << "Person 2's distance to dest: " << pathLength(G, P2Path);
        cout << " miles" << endl;
        cout << "Path: ";
        outputPath(P2Path);
      }
    }

    //
    // another navigation?
    //
    cout << endl;
    cout << "Enter person 1's building (partial name or abbreviation), or #> ";
    getline(cin, person1Building);
  }
}
