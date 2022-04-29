#include "trojanmap.h"
#include <string>
#include <utility>
#include <ctype.h>
#include <iostream>
#include <type_traits>
//-----------------------------------------------------
// TODO: Student should implement the following:
//-----------------------------------------------------
/**
 * GetLat: Get the latitude of a Node given its id. If id does not exist, return -1.
 * 
 * @param  {std::string} id : location id
 * @return {double}         : latitude
 */
double TrojanMap::GetLat(const std::string& id) {
    return data[id].lat;
}

/**
 * GetLon: Get the longitude of a Node given its id. If id does not exist, return -1.
 * 
 * @param  {std::string} id : location id
 * @return {double}         : longitude
 */
double TrojanMap::GetLon(const std::string& id) { 
    return data[id].lon;
}

/**
 * GetName: Get the name of a Node given its id. If id does not exist, return "NULL".
 * 
 * @param  {std::string} id : location id
 * @return {std::string}    : name
 */
std::string TrojanMap::GetName(const std::string& id) { 
  if (data.find(id) == data.end()) {
    return "NULL";
  }
  else {
    return data[id].name;
  }
}

/**
 * GetNeighborIDs: Get the neighbor ids of a Node. If id does not exist, return an empty vector.
 * 
 * @param  {std::string} id            : location id
 * @return {std::vector<std::string>}  : neighbor ids
 */
std::vector<std::string> TrojanMap::GetNeighborIDs(const std::string& id) {
  if (data.find(id) == data.end()) {
    return std::vector <std::string>();
  }
  else {
    return data[id].neighbors;
  }
}

/**
 * GetID: Given a location name, return the id. 
 * If the node does not exist, return an empty string. 
 *
 * @param  {std::string} name          : location name
 * @return {int}  : id
 */
std::string TrojanMap::GetID(const std::string& name) {
  std::string res = "";
  for (auto &it : data) {
    if (it.second.name == name) {
      res = it.first;
      break;
    }
  }
  return res;
}

/**
 * GetPosition: Given a location name, return the position. If id does not exist, return (-1, -1).
 *
 * @param  {std::string} name          : location name
 * @return {std::pair<double,double>}  : (lat, lon)
 */
std::pair<double, double> TrojanMap::GetPosition(std::string name) {
  std::pair<double, double> results(-1, -1);
  if(name == "") { //if empty name then return empty 
    return results;
  }
  for (auto id : data) {
    if (name == id.second.name) {
      results.first = GetLat(id.first);
      results.second = GetLon(id.first);
      break;
    }
  }
  return results;
}


/**
 * CalculateEditDistance: Calculate edit distance between two location names
 * 
 */
int TrojanMap::CalculateEditDistance(std::string a, std::string b){
  std::vector<std::vector<int>> memo(a.length()+1, std::vector<int> (b.length()+1,0));

  for(int i = 0; i <= b.length(); i++) {
    memo[0][i] = i;
  }
  
  for(int i = 0; i <= a.length(); i++) {
    memo[i][0] = i;
  }
  
  for(int i = 1; i <= a.length(); i++) {
    for(int j = 1; j <= b.length(); j++) {
      if(a[i-1] == b[j-1]) {
          memo[i][j] = memo[i-1][j-1];
      } else {
        int replace = memo[i-1][j-1] +1;
        int insert  = memo[i][j-1] + 1;
        int del = memo[i-1][j] + 1;
        int min = std::min(std::min(replace, insert), del);
        memo[i][j] = min;
      }
    }
  }
  
  return memo[a.length()][b.length()];
}

/**
 * FindClosestName: Given a location name, return the name with smallest edit distance.
 *
 * @param  {std::string} name          : location name
 * @return {std::string} tmp           : similar name
 */
std::string TrojanMap::FindClosestName(std::string name) {
  std::string tmp = "";  
  bool found= false;
  int min_dis = INT_MAX;
  
  for (auto it = data.begin(); it != data.end() && !found; ++it)
  {
    std::string currStr = it->second.name;
    if (currStr != ""){
      if (TrojanMap::CalculateEditDistance(name, currStr) < min_dis)
      {
        tmp = currStr;
        min_dis = TrojanMap::CalculateEditDistance(name, currStr);
      }
    }
  }
  return tmp;
}


/**
 * Autocomplete: Given a parital name return all the possible locations with
 * partial name as the prefix. The function should be case-insensitive.
 *
 * @param  {std::string} name          : partial name
 * @return {std::vector<std::string>}  : a vector of full names
 */
std::vector<std::string> TrojanMap::Autocomplete(std::string name){
  std::vector<std::string> results;
  std::transform(name.begin(), name.end(), name.begin(), [](unsigned char c){return std::tolower(c);});
  std::string temp_val ="";
  std::unordered_map<std::string,Node>:: iterator it = data.begin();
  while(it != data.end()){
    temp_val = it->second.name;
    std::transform(temp_val.begin(), temp_val.end(), temp_val.begin(), [](unsigned char c){return std::tolower(c);});
    if(name.compare(temp_val.substr(0,name.size()))==0)
      results.push_back(it->second.name);
    it++;
  }
  return results;
}

/**
 * CalculateDistance: Get the distance between 2 nodes. 
 * 
 * @param  {std::string} a  : a_id
 * @param  {std::string} b  : b_id
 * @return {double}  : distance in mile
 */
double TrojanMap::CalculateDistance(const std::string &a_id, const std::string &b_id) {
  // Do not change this function
  Node a = data[a_id];
  Node b = data[b_id];
  double dlon = (b.lon - a.lon) * M_PI / 180.0;
  double dlat = (b.lat - a.lat) * M_PI / 180.0;
  double p = pow(sin(dlat / 2),2.0) + cos(a.lat * M_PI / 180.0) * cos(b.lat * M_PI / 180.0) * pow(sin(dlon / 2),2.0);
  double c = 2 * asin(std::min(1.0,sqrt(p)));
  return c * 3961;
}

/**
 * CalculatePathLength: Calculates the total path length for the locations inside the vector.
 * 
 * @param  {std::vector<std::string>} path : path
 * @return {double}                        : path length
 */
double TrojanMap::CalculatePathLength(const std::vector<std::string> &path) {
  // Do not change this function
  double sum = 0;
  for (int i = 0;i < int(path.size())-1; i++) {
    sum += CalculateDistance(path[i], path[i+1]);
  }
  return sum;
}

/**
 * CalculateShortestPath_Dijkstra: Given 2 locations, return the shortest path which is a
 * list of id. Hint: Use priority queue.
 *
 * @param  {std::string} location1_name     : start
 * @param  {std::string} location2_name     : goal
 * @return {std::vector<std::string>}       : path
 */
std::vector<std::string> TrojanMap::CalculateShortestPath_Dijkstra(std::string location1_name, std::string location2_name) {
  std::vector<std::string> path;

  std::unordered_map<std::string, double> distance; 
  std::unordered_map<std::string, std::string> predecessor;
  std::unordered_map<std::string, bool> visited;

  for (auto node : data) { 
    distance[node.first] = INT_MAX;
  }

  for (auto node : data) { 
    visited[node.first] = false;
  }

  std::string location1_id = GetID(location1_name); 
  std::string location2_id = GetID(location2_name); 

  if (location1_id == location2_id) { 
    return path;
  }

  std::priority_queue<std::pair<double, std::string>, std::vector<std::pair<double, std::string>>, std::greater<std::pair<double, std::string>>> pq; 

  pq.push(std::make_pair(0, location1_id)); 

  distance[location1_id] = 0; 

  while (!pq.empty()) {
    std::string current_id = pq.top().second; 
    pq.pop();
    if (current_id == location2_id) { 
      break;
    }

    if (visited[current_id]) { 
      continue;
    }

    visited[current_id] = true; 

    std::vector<std::string> neighbors = GetNeighborIDs(current_id);
    
    for (auto neighbor : neighbors) {
      double distance_to_neighbor = CalculateDistance(current_id, neighbor);

      if (distance[current_id] + distance_to_neighbor < distance[neighbor]) {
        distance[neighbor] = distance[current_id] + distance_to_neighbor;
        predecessor[neighbor] = current_id;
        pq.push(std::make_pair(distance[neighbor], neighbor));
      }
    }
  }

  std::string current_id = location2_id;
  while (current_id != location1_id) {
    path.push_back(current_id);
    current_id = predecessor[current_id];
  }

  path.push_back(location1_id);
  
  std::reverse(path.begin(), path.end());
  return path;
}

/**
 * CalculateShortestPath_Bellman_Ford: Given 2 locations, return the shortest path which is a
 * list of id. Hint: Do the early termination when there is no change on distance.
 *
 * @param  {std::string} location1_name     : start
 * @param  {std::string} location2_name     : goal
 * @return {std::vector<std::string>}       : path
 */
std::vector<std::string> TrojanMap::CalculateShortestPath_Bellman_Ford(
    std::string location1_name, std::string location2_name){

  std::vector<std::string> path;
  std::unordered_map<std::string, double> distance;
  std::unordered_map<std::string, std::string> predecessor;
  for (auto node : data) {
    distance[node.first] = INT_MAX;
  }
  std::string location1_id = GetID(location1_name);
  std::string location2_id = GetID(location2_name);

  
  if (location1_id == location2_id) {
    return path;
  }

  distance[location1_id] = 0;
  int iterations = data.size()-1;
  std::string current_id = location1_id;
  bool done = true;

  for (int i = 0; i < iterations; i++) {
    done = true;
    for (auto node: data) {
      std::vector<std::string> neighbors = GetNeighborIDs(node.first);
      for (auto neighbor: neighbors) {
        double distance_to_neighbor = CalculateDistance(node.first, neighbor);
        if (distance[node.first] + distance_to_neighbor < distance[neighbor]) {
          distance[neighbor] = distance[node.first] + distance_to_neighbor;
          predecessor[neighbor] = node.first;
          done = false;
        }
      }
    }
    if (done) {
      break;
    }
  }

  current_id = location2_id;
  while (current_id != location1_id) {
    path.push_back(current_id);
    current_id = predecessor[current_id];
    }

  path.push_back(location1_id);  
  std::reverse(path.begin(), path.end());
  return path;
}

void TrojanMap::TravellingTrojan_helper(std::vector<std::string> &location_ids,std::vector<std::vector<double>> &weights,
    std::vector<std::vector<std::string>> &path,double &minDist,
    std::vector<int> &currentPath,double currDist,std::unordered_set<int> &seen, bool is_bruteforce)
{

  // Arrive at the leaf
  if (currentPath.size() == location_ids.size())
  {
    double finalDist = currDist + weights[currentPath.back()][0];
    if (finalDist < minDist)
    {
      std::vector<std::string> tempPath;
      for (auto i : currentPath)
        tempPath.push_back(location_ids[i]);
      tempPath.push_back(location_ids[0]);
      path.push_back(std::move(tempPath));
      minDist = finalDist;
    }
  }

  // Early proning
  if (!is_bruteforce){
    if (currDist >= minDist)
      return;
  }

  for (auto i = 1; i < location_ids.size(); i++)
  {
    if (seen.count(i) == 0)
    {
      seen.insert(i);
      double deltaDist = weights[currentPath.back()][i];
      currentPath.push_back(i);
      TravellingTrojan_helper(location_ids, weights, path, minDist, currentPath, currDist + deltaDist, seen, is_bruteforce);
      currentPath.pop_back();
      seen.erase(i);
    }
  }
}

/**
 * Travelling salesman problem: Given a list of locations, return the shortest
 * path which visit all the places and back to the start point.
 *
 * @param  {std::vector<std::string>} input : a list of locations needs to visit
 * @return {std::pair<double, std::vector<std::vector<std::string>>} : a pair of total distance and the all the progress to get final path
 */
std::pair<double, std::vector<std::vector<std::string>>> TrojanMap::TravellingTrojan_Brute_force(
std::vector<std::string> location_ids) {

  std::pair<double, std::vector<std::vector<std::string>>> records;
  if (location_ids.size() < 2)
    return records;

  // Initialize a vector to store the distance from start to each node
  std::vector<std::vector<double>> weight(location_ids.size(), std::vector<double>(location_ids.size()));

  // Initialize a vector to store the optimal path
  std::vector<std::vector<std::string>> path;

  // Initialize minimum distance
  double minDist = DBL_MAX;

  // Initialize a vector to store the path
  std::vector<int> currentPath;

  std::unordered_set<int> seen;

  // Add the start location to the path
  currentPath.push_back(0);

  seen.insert(0);

  // Initialize the weight vector with the distance from start to each node
  for (auto i = 0; i < location_ids.size(); i++)
  {
    for (auto j = i + 1; j < location_ids.size(); j++)
    {
      weight[i][j] = CalculateDistance(location_ids[i], location_ids[j]);
      weight[j][i] = weight[i][j];
    }
  }
  
  TravellingTrojan_helper(location_ids, weight, path, minDist, currentPath, 0, seen, true);

  // Update the minimum distance
  records.first = minDist;

  // Push back optimal path
  records.second = path;

  return records;
}

std::pair<double, std::vector<std::vector<std::string>>> TrojanMap::TravellingTrojan_Backtracking(
    std::vector<std::string> location_ids)
  {
    std::pair<double, std::vector<std::vector<std::string>>> records;
    if (location_ids.size() < 2)
      return records;

    std::vector<std::vector<double>> weight(location_ids.size(), std::vector<double>(location_ids.size()));
    for (auto i = 0; i < location_ids.size(); i++)
    {
      for (auto j = i + 1; j < location_ids.size(); j++)
      {
        weight[i][j] = CalculateDistance(location_ids[i], location_ids[j]);
        weight[j][i] = weight[i][j];
      }
    }
    std::vector<std::vector<std::string>> paths;
    double minDist = DBL_MAX;
    std::vector<int> current_path;
    std::unordered_set<int> seen;
    current_path.push_back(0);
    seen.insert(0);

    TravellingTrojan_helper(location_ids, weight, paths, minDist, current_path, 0, seen, false);
    return std::pair<double, std::vector<std::vector<std::string>>>(minDist, paths);
}

std::pair<double, std::vector<std::vector<std::string>>> TrojanMap::TravellingTrojan_2opt(
      std::vector<std::string> location_ids){
  
  std::pair<double, std::vector<std::vector<std::string>>> records;
if(location_ids.size()<2) return records;

  std::vector<std::vector<std::string>> allPath;
  std::vector<std::string> currPath;
  for(std::string &location:location_ids)
    currPath.push_back(location);
  currPath.push_back(location_ids[0]);
  double minDist = CalculatePathLength(currPath);
  allPath.push_back(currPath);

  bool stop = false;
  while(!stop){
    stop = true;
    // currPath has at least three nodes
    for(auto i=1;i<currPath.size()-1;i++){
      for(auto k=i+1;k<currPath.size();k++){
        std::reverse(currPath.begin()+i,currPath.begin()+k);
        double currDist = CalculatePathLength(currPath);
        if(currDist<minDist){
          minDist = currDist;
          allPath.push_back(currPath);
          stop = false;
        }else{
          // Recover the change on the original path
          std::reverse(currPath.begin()+i,currPath.begin()+k);
        }
      }
    }
  }
  return std::pair<double, std::vector<std::vector<std::string>>>(minDist,allPath);
}

/**
 * Given CSV filename, it read and parse locations data from CSV file,
 * and return locations vector for topological sort problem.
 *
 * @param  {std::string} locations_filename     : locations_filename
 * @return {std::vector<std::string>}           : locations 
 */
std::vector<std::string> TrojanMap::ReadLocationsFromCSVFile(std::string locations_filename){
  std::vector<std::string> location_names_from_csv;
  std::fstream f;
  std::string line;

  f.open(locations_filename, std::ios::in); // Open the file
  if(!f.is_open()) // If the file is already open
  {
    std::cout<<"Couldn't open "<< locations_filename << std::endl;
    return location_names_from_csv;
  }
  
  getline(f, line); // Call getline to ignore the header

  while(getline(f,line)) // Read the rest of the lines
  {
    line.erase(std::remove(line.begin(),line.end(),','), line.end());
    if(line!="")
      location_names_from_csv.push_back(line);
  }
  f.close();
  return location_names_from_csv;
}

/**
 * Given CSV filenames, it read and parse dependencise data from CSV file,
 * and return dependencies vector for topological sort problem.
 *
 * @param  {std::string} dependencies_filename     : dependencies_filename
 * @return {std::vector<std::vector<std::string>>} : dependencies
 */
std::vector<std::vector<std::string>> TrojanMap::ReadDependenciesFromCSVFile(std::string dependencies_filename){
  std::vector<std::vector<std::string>> dependencies_from_csv;
  std::fstream f;
  std::string line;

  f.open(dependencies_filename, std::ios::in);
  if(!f.is_open())
  {
    std::cout<<"Couldn't open "<< dependencies_filename << std::endl;
    return dependencies_from_csv;
  }
  
  getline(f, line);
  while (getline(f, line)) {
    std::string firstpos , secondpos;
    auto pos = line.find(',');
    if(pos==-1 || pos==0 || pos==line.size()-1) continue;
    firstpos = line.substr(0,pos);
    secondpos = line.substr(pos+1);
    secondpos.erase(std::remove(secondpos.begin(),secondpos.end(),','), secondpos.end());
    dependencies_from_csv.push_back({firstpos,secondpos});
  }
  f.close();
  return dependencies_from_csv;
}

/**
 * DeliveringTrojan: Given a vector of location names, it should return a sorting of nodes
 * that satisfies the given dependencies. If there is no way to do it, return a empty vector.
 *
 * @param  {std::vector<std::string>} locations                     : locations
 * @param  {std::vector<std::vector<std::string>>} dependencies     : prerequisites
 * @return {std::vector<std::string>} results                       : results
 */
std::vector<std::string> TrojanMap::DeliveringTrojan(std::vector<std::string> &locations,
                                                     std::vector<std::vector<std::string>> &dependencies){
  std::vector<std::string> result;
  std::unordered_map<std::string, bool> visited; // Map to store the visited nodes
  std::unordered_map<std::string, std::vector<std::string>> dependency_map; // Map to store the dependencies
  for(auto location: locations) {
    std::vector<std::string> temp;
    dependency_map[location] = temp;
  }

  for (auto location : locations) { // visited map is initialized
    visited[location] = false;
  }

  for (auto &dependency : dependencies) { // Create edge map
    dependency_map[dependency[0]].push_back(dependency[1]);
  }

  for (auto location : locations) { // topological sort helper function on unvisited nodes
    if (!visited[location]) {
      TopologicalSort(location, dependency_map, visited, result);
    }
  }
  std::unique(result.begin(),result.end());
  std::reverse(result.begin(), result.end());  // Reverse the result
  return result;                                         
}

// Helper function for topological sort
void TrojanMap::TopologicalSort(std::string &location,
                                std::unordered_map<std::string, std::vector<std::string>> &dependency_map,
                                std::unordered_map<std::string, bool> &visited,
                                std::vector<std::string> &result) {
  visited[location] = true; // dependency is visited
  // Iterate through all the dependencies
  for (auto &dependency : dependency_map[location]) {
    if (!visited[dependency]) { // If the dependency is not visited
      TopologicalSort(dependency, dependency_map, visited, result); // topological sort
    }
  }
  result.push_back(location); // Add the location to the result
}

/**
 * inSquare: Give a id retunr whether it is in square or not.
 *
 * @param  {std::string} id            : location id
 * @param  {std::vector<double>} square: four vertexes of the square area
 * @return {bool}                      : in square or not
 */
bool TrojanMap::inSquare(std::string id, std::vector<double> &square) {
  bool is_square = false;
  if(data[id].lon>=square[0] && data[id].lon<=square[1] && data[id].lat<=square[2] && data[id].lat>=square[3]){
    is_square = true;
  }
  return is_square;
}

/**
 * GetSubgraph: Give four vertexes of the square area, return a list of location ids in the squares
 *
 * @param  {std::vector<double>} square         : four vertexes of the square area
 * @return {std::vector<std::string>} subgraph  : list of location ids in the square
 */
std::vector<std::string> TrojanMap::GetSubgraph(std::vector<double> &square) {
  // include all the nodes in subgraph
  std::vector<std::string> subgraph;

  for(auto node:data){
    if(inSquare(node.first,square)){
      subgraph.push_back(node.first);
    }
  }
  return subgraph;
}

/**
 * Has Cycle: Returns true if it has cycle
 * 
 * @param {std::string} current_id: current node id
 * @param {std::string} parent_id: parent node id
 * @param {std::vector<std::string>} subgraph: list of location ids in the square
 * @param {std::vector<double>} square: four vertexes of the square area
 * @param {std::map<std::string, std::string>} predecessor: predecessor of the current node
 * @return {bool}: whether there is a cycle or not
 */
bool TrojanMap::hasCycle(std::string current_id, std::map<std::string, bool> &visited, std::string parent_id, std::vector<double> &square,std::map<std::string, std::string>& predecessor)
{
  visited[current_id] = true; // Mark the current node as visited
  if(inSquare(current_id, square)) 
  {
    // Find all the vertices which are not visited and are adjacent to the current node
    std::vector<std::string> neighbors = GetNeighborIDs(current_id);
    for(auto neighbor: neighbors)
    {
      predecessor[neighbor] = current_id; // record current node as predecessor node of these neighbor nodes
      if(!visited[neighbor] && inSquare(neighbor, square)) // If the neighbor is not visited, call the helper function recursively
      {
        if(hasCycle(neighbor, visited, current_id, square, predecessor))
        {
          return true;
        }
      }
      else if(inSquare(neighbor, square) && visited[neighbor] && neighbor != parent_id) { // If neighbor is visited and is not parent of the current node, there is a cycle
        return true;
      }
    }
  }
  return false;
}
/**
 * Cycle Detection: Given four points of the square-shape subgraph, return true if there
 * is a cycle path inside the square, false otherwise.
 * 
 * @param {std::vector<std::string>} subgraph: list of location ids in the square
 * @param {std::vector<double>} square: four vertexes of the square area
 * @return {bool}: whether there is a cycle or not
 */
bool TrojanMap::CycleDetection(std::vector<std::string> &subgraph, std::vector<double> &square) {
  std::map<std::string, bool> visited; //to hold if node is visited
  std::map<std::string, std::string> predecessor; //to hold parent_ids
  
  //initialize with visited as false
  for(auto id: subgraph)
  {
    visited[id] = false;
  }

  for(auto node: subgraph)
  {
    if(!visited[node]) //check only if node is not visited
    {
      if(hasCycle(node, visited, node, square, predecessor)) //check for cycle
      {
        return true;
      }
    }
  }
  return false;
}


/**
 * FindNearby: Given a class name C, a location name L and a number r, 
 * find all locations in class C on the map near L with the range of r and return a vector of string ids
 * 
 * @param {std::string} className: the name of the class
 * @param {std::string} locationName: the name of the location
 * @param {int} r: search radius
 * @param {int} k: search numbers
 * @return {std::vector<std::string>}: location name that meets the requirements
 */
std::vector<std::string> TrojanMap::FindNearby(std::string attributesName, std::string name, double r, int k) {
  std::vector<std::string> res;

  // ID of the source
  std::string source = GetID(name);
  // priority queue to store the distance from the source to the node
  std::priority_queue<std::pair<double, std::string>, std::vector<std::pair<double, std::string>>, std::greater<std::pair<double, std::string>>> priority_q;

  
  for (auto &loc: data) {
    if (loc.first == source) { // Skip the source
      continue;
    }
    if (std::find(loc.second.attributes.begin(), loc.second.attributes.end(), attributesName) != loc.second.attributes.end()) {
      priority_q.push(std::make_pair(CalculateDistance(source, loc.first), loc.first));
    }
  }

  for (int i = 0; i < k; i++) {
    if (priority_q.empty()) {
      return res;
    }
    std::string id = priority_q.top().second;
    priority_q.pop();
    if (CalculateDistance(source, id) <= r) {
      res.push_back(id);
    }
  }
  return res;
}

/**
 * CreateGraphFromCSVFile: Read the map data from the csv file
 * 
 */
void TrojanMap::CreateGraphFromCSVFile() {
  // Do not change this function
  std::fstream fin;
  fin.open("src/lib/data.csv", std::ios::in);
  std::string line, word;

  getline(fin, line);
  while (getline(fin, line)) {
    std::stringstream s(line);

    Node n;
    int count = 0;
    while (getline(s, word, ',')) {
      word.erase(std::remove(word.begin(), word.end(), '\''), word.end());
      word.erase(std::remove(word.begin(), word.end(), '"'), word.end());
      word.erase(std::remove(word.begin(), word.end(), '{'), word.end());
      word.erase(std::remove(word.begin(), word.end(), '}'), word.end());
      if (count == 0)
        n.id = word;
      else if (count == 1)
        n.lat = stod(word);
      else if (count == 2)
        n.lon = stod(word);
      else if (count == 3)
        n.name = word;
      else {
        word.erase(std::remove(word.begin(), word.end(), ' '), word.end());
        if (isalpha(word[0]))
          n.attributes.insert(word);
        if (isdigit(word[0]))
          n.neighbors.push_back(word);
      }
      count++;
    }
    data[n.id] = n;
  }
  fin.close();
}
