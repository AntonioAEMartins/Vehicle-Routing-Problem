#ifndef BASE_H
#define BASE_H

#include <iostream>
#include <fstream>
#include <map>
#include <vector>
#include <set>

void load_graph(const std::string &filename, std::map<int, int> &nodes, std::map<std::pair<int, int>, int> &distances);
std::vector<std::vector<int>> generatePermutations(const std::map<int, int> &locations);
std::vector<std::vector<int>> generatePermutationsParallel(const std::map<int, int> &locations);
std::vector<std::vector<int>> generatePermutationsParallelOptimized(const std::map<int, int> &locations);
void printPermutations(const std::vector<std::vector<int>> &permutations);
void printPaths(const std::vector<std::vector<int>> &possiblePaths);
void printPath(const std::vector<int> &path, const std::string &text = "", int cost = 0);
std::vector<std::vector<int>> generatePossiblePaths(std::vector<std::vector<int>> permutations, const std::map<std::pair<int, int>, int> &distances, const std::map<int, int> &nodes, int maxCapacity);
std::vector<std::vector<int>> generatePossiblePathsParallel(const std::vector<std::vector<int>> &permutations, const std::map<std::pair<int, int>, int> &distances, const std::map<int, int> &nodes, int maxCapacity);
int findClosestNode(int node, const std::set<int> &unvisitedNodes, const std::map<int, int> &nodes, const std::map<std::pair<int, int>, int> &distances);

#endif // BASE_H
