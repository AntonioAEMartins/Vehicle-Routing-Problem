#include <iostream>
#include <fstream>
#include <sstream>
#include <map>
#include <vector>
#include <set>
#include <climits>
#include <omp.h>
#include "base.h"

using namespace std;
using namespace std::chrono;

/**
 * @brief Finds the best path with the minimum cost from a set of possible paths.
 * 
 * This function iterates through all possible paths and calculates their respective costs
 * using the provided distances. It then selects the path with the minimum cost.
 * 
 * @param possiblePaths A vector of vectors where each inner vector represents a possible path as a sequence of nodes.
 * @param distances A map that holds the distances between pairs of nodes.
 * @param cost A reference to an integer where the cost of the best path will be stored.
 * @return A vector of integers representing the nodes in the best path with the minimum cost.
 */
vector<int> findBestPath(vector<vector<int>> possiblePaths, map<pair<int, int>, int> &distances, int &cost)
{
    vector<int> bestPath;
    int minCost = INT_MAX;
    int numPossiblePaths = possiblePaths.size();

    for (int i = 0; i < possiblePaths.size(); i++)
    {
        int pathCost = 0;
        for (int j = 0; j < possiblePaths[i].size() - 1; j++)
        {
            int from = possiblePaths[i][j];
            int to = possiblePaths[i][j + 1];
            int cost = distances.at({from, to});
            pathCost += cost;
        }
        if (pathCost < minCost)
        {
            minCost = pathCost;
            bestPath = possiblePaths[i];
        }
    }
    cost = minCost;
    return bestPath;
}

/**
 * @brief Finds the best path with the minimum cost from a set of possible paths using parallel processing.
 * 
 * This function parallelizes the search for the best path using OpenMP, allowing for the evaluation of multiple paths concurrently.
 * It calculates the cost of each path and selects the path with the minimum cost.
 * 
 * @param possiblePaths A vector of vectors where each inner vector represents a possible path as a sequence of nodes.
 * @param distances A map that holds the distances between pairs of nodes.
 * @param cost A reference to an integer where the cost of the best path will be stored.
 * @return A vector of integers representing the nodes in the best path with the minimum cost.
 */
vector<int> findBestPathParallel(vector<vector<int>> possiblePaths, map<pair<int, int>, int> &distances, int &cost)
{
    vector<int> bestPath;
    int minCost = INT_MAX;

#pragma omp parallel for
    for (int i = 0; i < possiblePaths.size(); i++)
    {
        int pathCost = 0;
        for (int j = 0; j < possiblePaths[i].size() - 1; j++)
        {
            int from = possiblePaths[i][j];
            int to = possiblePaths[i][j + 1];
            int cost = distances.at({from, to});
            pathCost += cost;
        }
        if (pathCost < minCost)
        {
            minCost = pathCost;
            bestPath = possiblePaths[i];
        }
    }
    cost = minCost;
    return bestPath;
}

/**
 * @brief Performs a Nearest Neighbor search for the Traveling Salesman Problem (TSP).
 * 
 * This function constructs a path by visiting the nearest unvisited node and returns to the starting node when necessary,
 * considering the capacity constraints of the nodes.
 * 
 * @param distances A map where keys are pairs of node indices representing edges and values are the corresponding distances.
 * @param nodes A map where keys are node indices and values are the capacities of the nodes.
 * @param cost An integer reference that will be set to the total cost of the path found.
 * @param maxCapacity The maximum capacity constraint for the nodes in the path.
 * @return A vector of integers representing the path starting and ending at node 0.
 */
vector<int> nearestNeighborSearch(map<pair<int, int>, int> &distances, map<int, int> &nodes, int &cost, int maxCapacity)
{
    vector<int> path;
    cost = 0;
    int capacity = 0;
    int current = 0;
    path.push_back(current);

    set<int> unvisitedNodes;
    for (const auto &node : nodes)
    {
        if (node.first != 0)
        {
            unvisitedNodes.insert(node.first);
        }
    }

    // Executar enquanto ainda houver nós não visitados
    for (size_t i = 0; !unvisitedNodes.empty(); ++i)
    {
        int nearestNode = findClosestNode(current, unvisitedNodes, nodes, distances);

        if (nearestNode != -1 && capacity + nodes.at(nearestNode) <= maxCapacity)
        {
            path.push_back(nearestNode);
            cost += distances.at({current, nearestNode});
            capacity += nodes.at(nearestNode);
            current = nearestNode;
            unvisitedNodes.erase(nearestNode);
        }
        else
        {
            path.push_back(0);
            cost += distances.at({current, 0});
            current = 0;
            capacity = 0;
        }
    }

    if (current != 0)
    {
        path.push_back(0);
        cost += distances.at({current, 0});
    }

    return path;
}

/**
 * @brief Performs a parallelized Nearest Neighbor search for the Traveling Salesman Problem (TSP).
 * 
 * This function parallelizes the Nearest Neighbor search using OpenMP to potentially speed up the computation.
 * The parallelization focuses on finding the nearest node in parallel.
 * 
 * @param distances A map where keys are pairs of node indices representing edges and values are the corresponding distances.
 * @param nodes A map where keys are node indices and values are the capacities of the nodes.
 * @param cost An integer reference that will be set to the total cost of the path found.
 * @param maxCapacity The maximum capacity constraint for the nodes in the path.
 * @return A vector of integers representing the path starting and ending at node 0.
 */
vector<int> nearestNeighborSearchParallel(map<pair<int, int>, int> &distances, map<int, int> &nodes, int &cost, int maxCapacity)
{
    vector<int> path;
    cost = 0;
    int capacity = 0;
    int current = 0;
    path.push_back(current);

    set<int> unvisitedNodes;
    for (const auto &node : nodes)
    {
        if (node.first != 0)
        {
            unvisitedNodes.insert(node.first);
        }
    }

#pragma omp parallel
    {
#pragma omp single
        {
            for (size_t i = 0; !unvisitedNodes.empty(); ++i)
            {
                int nearestNode = -1;

#pragma omp task shared(nearestNode)
                {
                    nearestNode = findClosestNode(current, unvisitedNodes, nodes, distances);
                }

#pragma omp taskwait
                if (nearestNode != -1 && capacity + nodes.at(nearestNode) <= maxCapacity)
                {
#pragma omp critical
                    {
                        path.push_back(nearestNode);
                        cost += distances.at({current, nearestNode});
                        capacity += nodes.at(nearestNode);
                        current = nearestNode;
                        unvisitedNodes.erase(nearestNode);
                    }
                }
                else
                {
#pragma omp critical
                    {
                        path.push_back(0);
                        cost += distances.at({current, 0});
                        current = 0;
                        capacity = 0;
                    }
                }
            }

            if (current != 0)
            {
#pragma omp critical
                {
                    path.push_back(0);
                    cost += distances.at({current, 0});
                }
            }
        }
    }

    return path;
}

/**
 * @brief Main function that executes various algorithms and measures their performance.
 * 
 * This function loads graph data from a file, generates permutations and possible paths, and performs both serial and parallel
 * algorithms for finding the best path and Nearest Neighbor search. It measures and prints the execution times for different
 * stages of the algorithm.
 * 
 * @return An integer indicating the exit status of the program.
 */
int main()
{
    int maxCapacity = 10;
    map<int, int> nodes;
    map<pair<int, int>, int> distances;
    load_graph("../grafo.txt", nodes, distances);

    // cout << "--- Times ---" << endl;

    auto start = high_resolution_clock::now();
    vector<vector<int>> permutations = generatePermutations(nodes);
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(stop - start);
    cout << "Permutations (S) " << duration.count() << " milli." << endl;

    start = high_resolution_clock::now();
    vector<vector<int>> permutationsParallel = generatePermutationsParallelOptimized(nodes);
    stop = high_resolution_clock::now();
    duration = duration_cast<milliseconds>(stop - start);
    cout << "Permutations (P) " << duration.count() << " milli." << endl;

    start = high_resolution_clock::now();
    vector<vector<int>> possiblePaths = generatePossiblePaths(permutations, distances, nodes, maxCapacity);
    stop = high_resolution_clock::now();
    duration = duration_cast<milliseconds>(stop - start);
    cout << "Paths (S) " << duration.count() << " milli." << endl;

    start = high_resolution_clock::now();
    vector<vector<int>> possiblePathsParallel = generatePossiblePathsParallel(permutations, distances, nodes, maxCapacity);
    stop = high_resolution_clock::now();
    duration = duration_cast<milliseconds>(stop - start);
    cout << "Paths (P) " << duration.count() << " milli." << endl;

    int costBestPath = 0;
    start = high_resolution_clock::now();
    vector<int> bestPath = findBestPath(possiblePaths, distances, costBestPath);
    stop = high_resolution_clock::now();
    duration = duration_cast<milliseconds>(stop - start);
    cout << "Best Path (S) " << duration.count() << " milli." << endl;

    int costBestPathParallel = 0;
    start = high_resolution_clock::now();
    vector<int> bestPathParalles = findBestPathParallel(possiblePaths, distances, costBestPathParallel);
    stop = high_resolution_clock::now();
    duration = duration_cast<milliseconds>(stop - start);
    cout << "Best Path (P)  " << duration.count() << " milli." << endl;

    int costNearestNeighbor = 0;
    start = high_resolution_clock::now();
    vector<int> nearestNeighborPath = nearestNeighborSearch(distances, nodes, costNearestNeighbor, maxCapacity);
    stop = high_resolution_clock::now();
    duration = duration_cast<milliseconds>(stop - start);
    cout << "Nearest Neighbor (S) " << duration.count() << " milli." << endl;

    int costNearestNeighborParallel = 0;
    start = high_resolution_clock::now();
    vector<int> nearestNeighborPathParallel = nearestNeighborSearchParallel(distances, nodes, costNearestNeighborParallel, maxCapacity);
    stop = high_resolution_clock::now();
    duration = duration_cast<milliseconds>(stop - start);
    cout << "Nearest Neighbor (P) " << duration.count() << " milli." << endl;

    printPath(bestPath, "Global (S)", costBestPath);
    printPath(bestPathParalles, "Global (P)", costBestPathParallel);
    printPath(nearestNeighborPath, "Nearest Neighbor (S)", costNearestNeighbor);
    printPath(nearestNeighborPathParallel, "Nearest Neighbor (P)", costNearestNeighborParallel);

    return 0;
}