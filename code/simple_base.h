#ifndef SIMPLE_BASE_H
#define SIMPLE_BASE_H

#include <iostream>
#include <fstream>
#include <vector>
#include <map>
#include <set>
#include <string>

// Function to load the graph from a file
void load_graph(const std::string &filename, std::map<int, int> &nodes, std::map<std::pair<int, int>, int> &distances);

// Function to generate all permutations of the locations (serial version)
std::vector<std::vector<int>> generatePermutations(const std::map<int, int> &locations);

// Function to print the permutations
void printPermutations(const std::vector<std::vector<int>> &permutations);

// Function to print all possible paths
void printPaths(const std::vector<std::vector<int>> &possiblePaths);

// Function to print a specific path
void printPath(const std::vector<int> &path, const std::string &text, int cost);

// Function to generate possible paths given the nodes, distances, and maximum capacity
std::vector<std::vector<int>> generatePossiblePaths(std::vector<std::vector<int>> permutations, const std::map<std::pair<int, int>, int> &distances, const std::map<int, int> &nodes, int maxCapacity);

// Function to find the closest node (serial version)
int findClosestNode(int node, const std::set<int> &unvisitedNodes, const std::map<int, int> &nodes, const std::map<std::pair<int, int>, int> &distances);

#endif // SIMPLE_BASE_H
