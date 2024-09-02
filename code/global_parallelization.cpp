#include <iostream>
#include <fstream>
#include <sstream>
#include <map>
#include <vector>
#include <set>
#include <omp.h>
#include "base.h"
#include <mpi.h>
#include <chrono>
#include <climits>

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
 * This function performs a parallel search to find the path with the minimum cost from a set of possible paths.
 * The distances are used to calculate the cost of each path in parallel.
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
 * @brief Finds the best path with the minimum cost from a set of possible paths using parallel processing with MPI.
 *
 * This function performs a parallel search to find the path with the minimum cost using MPI for distributed computing.
 * The distances are used to calculate the cost of each path in parallel across different MPI processes.
 *
 * @param possiblePaths A vector of vectors where each inner vector represents a possible path as a sequence of nodes.
 * @param distances A map that holds the distances between pairs of nodes.
 * @param cost A reference to an integer where the cost of the best path will be stored.
 * @param rank The rank of the current MPI process.
 * @param size The total number of MPI processes.
 * @return A vector of integers representing the nodes in the best path with the minimum cost.
 */

vector<int> findBestPathParallelMPI(vector<vector<int>> possiblePaths, map<pair<int, int>, int> &distances, int &cost, int rank, int size)
{
    vector<int> bestPath;
    int minCost = INT_MAX;

    int chunkSize = possiblePaths.size() / size;
    int start = rank * chunkSize;
    int end = (rank == size - 1) ? possiblePaths.size() : start + chunkSize;

    // Para evitar usar uma seção crítica, vamos usar variáveis privadas para cada thread
    int localMinCost = INT_MAX;
    vector<int> localBestPath;

#pragma omp parallel
    {
        int threadMinCost = INT_MAX;
        vector<int> threadBestPath;

#pragma omp for nowait
        for (int i = start; i < end; i++)
        {
            int pathCost = 0;
            for (int j = 0; j < possiblePaths[i].size() - 1; j++)
            {
                int from = possiblePaths[i][j];
                int to = possiblePaths[i][j + 1];
                int cost = distances.at({from, to});
                pathCost += cost;
            }

            if (pathCost < threadMinCost)
            {
                threadMinCost = pathCost;
                threadBestPath = possiblePaths[i];
            }
        }

#pragma omp critical
        {
            if (threadMinCost < localMinCost)
            {
                localMinCost = threadMinCost;
                localBestPath = threadBestPath;
            }
        }
    }

    // Usar MPI para reduzir os resultados dos diferentes processos
    MPI_Allreduce(&localMinCost, &minCost, 1, MPI_INT, MPI_MIN, MPI_COMM_WORLD);

    if (localMinCost == minCost)
    {
        bestPath = localBestPath;
    }

    // Broadcast do melhor caminho encontrado para todos os processos
    int pathSize = bestPath.size();
    MPI_Bcast(&pathSize, 1, MPI_INT, 0, MPI_COMM_WORLD);
    bestPath.resize(pathSize);
    MPI_Bcast(bestPath.data(), pathSize, MPI_INT, 0, MPI_COMM_WORLD);

    cost = minCost;
    return bestPath;
}

/**
 * @brief Finds a path using the nearest neighbor heuristic.
 *
 * This function finds a path starting from the initial node and repeatedly visits the nearest unvisited node
 * until all nodes are visited. It then returns to the starting node. The cost is calculated based on the distances.
 *
 * @param distances A map that holds the distances between pairs of nodes.
 * @param nodes A map of nodes with their capacities.
 * @param cost A reference to an integer where the cost of the path will be stored.
 * @param maxCapacity The maximum capacity allowed for the path.
 * @return A vector of integers representing the nodes in the path found using the nearest neighbor heuristic.
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
 * @brief Finds a path using the nearest neighbor heuristic with parallel processing.
 *
 * This function performs a parallel search using the nearest neighbor heuristic. It finds the nearest unvisited node
 * in parallel and updates the path and cost accordingly.
 *
 * @param distances A map that holds the distances between pairs of nodes.
 * @param nodes A map of nodes with their capacities.
 * @param cost A reference to an integer where the cost of the path will be stored.
 * @param maxCapacity The maximum capacity allowed for the path.
 * @return A vector of integers representing the nodes in the path found using the nearest neighbor heuristic in parallel.
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
 * @brief Finds a path using the nearest neighbor heuristic with parallel processing and MPI.
 *
 * This function performs a parallel search using both OpenMP and MPI to find a path based on the nearest neighbor heuristic.
 * It distributes the search process across multiple MPI processes and uses OpenMP to parallelize within each process.
 *
 * @param distances A map that holds the distances between pairs of nodes.
 * @param nodes A map of nodes with their capacities.
 * @param cost A reference to an integer where the cost of the path will be stored.
 * @param maxCapacity The maximum capacity allowed for the path.
 * @param rank The rank of the current MPI process.
 * @param size The total number of MPI processes.
 * @return A vector of integers representing the nodes in the path found using the nearest neighbor heuristic with MPI.
 */

vector<int> nearestNeighborSearchParallelMPI(map<pair<int, int>, int> &distances, map<int, int> &nodes, int &cost, int maxCapacity, int rank, int size)
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

    vector<int> globalPath(path.size());
    MPI_Allreduce(path.data(), globalPath.data(), path.size(), MPI_INT, MPI_SUM, MPI_COMM_WORLD);

    return globalPath;
}

/**
 * @brief Main function to execute the pathfinding algorithms.
 *
 * This function initializes MPI, loads the graph data, and executes various pathfinding algorithms including
 * sequential, parallel, and MPI-based approaches. It measures and prints the execution time for each algorithm.
 *
 * @param argc The number of command-line arguments.
 * @param argv The array of command-line arguments.
 * @return Returns 0 upon successful execution.
 */

int main(int argc, char **argv)
{
    MPI_Init(&argc, &argv);
    int rank, size;
    MPI_Comm_rank(MPI_COMM_WORLD, &rank);
    MPI_Comm_size(MPI_COMM_WORLD, &size);

    int maxCapacity = 10;
    map<int, int> nodes;
    map<pair<int, int>, int> distances;
    load_graph("../grafo.txt", nodes, distances);

    // cout << "--- Times ---" << endl;

    auto start = high_resolution_clock::now();
    vector<vector<int>> permutations = generatePermutations(nodes);
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(stop - start);
    if (rank == 0) cout << "Permutations (S) " << duration.count() << " milli." << endl;

    // start = high_resolution_clock::now();
    // vector<vector<int>> permutationsParallel = generatePermutationsParallelOptimized(nodes);
    // stop = high_resolution_clock::now();
    // duration = duration_cast<milliseconds>(stop - start);
    // if (rank == 0) cout << "Permutations (P) " << duration.count() << " milli." << endl;

    // start = high_resolution_clock::now();
    // vector<vector<int>> possiblePaths = generatePossiblePaths(permutations, distances, nodes, maxCapacity);
    // stop = high_resolution_clock::now();
    // duration = duration_cast<milliseconds>(stop - start);
    // if (rank == 0) cout << "Paths (S) " << duration.count() << " milli." << endl;

    start = high_resolution_clock::now();
    vector<vector<int>> possiblePathsParallel = generatePossiblePathsParallel(permutations, distances, nodes, maxCapacity);
    stop = high_resolution_clock::now();
    duration = duration_cast<milliseconds>(stop - start);
    if (rank == 0) cout << "Paths (P) " << duration.count() << " milli." << endl;

    int costBestPath = 0;
    start = high_resolution_clock::now();
    vector<int> bestPath = findBestPath(possiblePathsParallel, distances, costBestPath);
    stop = high_resolution_clock::now();
    duration = duration_cast<milliseconds>(stop - start);
    if (rank == 0) cout << "Best Path (S) " << duration.count() << " milli." << endl;

    int costBestPathParallel = 0;
    start = high_resolution_clock::now();
    vector<int> bestPathParallel = findBestPathParallel(possiblePathsParallel, distances, costBestPathParallel);
    stop = high_resolution_clock::now();
    duration = duration_cast<milliseconds>(stop - start);
    if (rank == 0) cout << "Best Path (P)  " << duration.count() << " milli." << endl;

    int costBestPathParallelMPI = 0;
    start = high_resolution_clock::now();
    vector<int> bestPathParallelMPI = findBestPathParallelMPI(possiblePathsParallel, distances, costBestPathParallelMPI, rank, size);
    stop = high_resolution_clock::now();
    duration = duration_cast<milliseconds>(stop - start);
    if (rank == 0) cout << "Best Path (MPI)  " << duration.count() << " milli." << endl;

    int costNearestNeighbor = 0;
    start = high_resolution_clock::now();
    vector<int> nearestNeighborPath = nearestNeighborSearch(distances, nodes, costNearestNeighbor, maxCapacity);
    stop = high_resolution_clock::now();
    duration = duration_cast<milliseconds>(stop - start);
    if (rank == 0) cout << "Nearest Neighbor (S) " << duration.count() << " milli." << endl;

    int costNearestNeighborParallel = 0;
    start = high_resolution_clock::now();
    vector<int> nearestNeighborPathParallel = nearestNeighborSearchParallel(distances, nodes, costNearestNeighborParallel, maxCapacity);
    stop = high_resolution_clock::now();
    duration = duration_cast<milliseconds>(stop - start);
    if (rank == 0) cout << "Nearest Neighbor (P) " << duration.count() << " milli." << endl;

    int costNearestNeighborParallelMPI = 0;
    start = high_resolution_clock::now();
    vector<int> nearestNeighborPathParallelMPI = nearestNeighborSearchParallelMPI(distances, nodes, costNearestNeighborParallelMPI, maxCapacity, rank, size);
    stop = high_resolution_clock::now();
    duration = duration_cast<milliseconds>(stop - start);
    if (rank == 0) cout << "Nearest Neighbor (MPI) " << duration.count() << " milli." << endl;

    if (rank == 0)
    {
        printPath(bestPath, "Global (S)", costBestPath);
        printPath(bestPathParallel, "Global (P)", costBestPathParallel);
        printPath(bestPathParallelMPI, "Global (MPI)", costBestPathParallelMPI);
        printPath(nearestNeighborPath, "Nearest Neighbor (S)", costNearestNeighbor);
        printPath(nearestNeighborPathParallel, "Nearest Neighbor (P)", costNearestNeighborParallel);
        printPath(nearestNeighborPathParallelMPI, "Nearest Neighbor (MPI)", costNearestNeighborParallelMPI);
    }

    MPI_Finalize();
    return 0;
}
