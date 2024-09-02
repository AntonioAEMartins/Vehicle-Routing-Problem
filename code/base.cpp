#include "base.h"
#include <sstream>
#include <algorithm>
#include <climits>
#include <omp.h>

/**
 * @brief Loads a graph from a file.
 *
 * This function reads a graph definition from a file and loads the nodes and distances into maps.
 * The graph file should contain the number of nodes, each node's ID and associated value,
 * followed by the number of edges and each edge's nodes and distance.
 *
 * @param filename The name of the file containing the graph definition.
 * @param nodes A map where the keys are node IDs and the values are associated values (e.g., capacity or demand).
 * @param distances A map where the keys are pairs of node IDs representing edges, and the values are the distances between those nodes.
 */

void load_graph(const std::string &filename, std::map<int, int> &nodes, std::map<std::pair<int, int>, int> &distances)
{
    std::ifstream infile(filename);
    if (!infile)
    {
        std::cerr << "Erro ao abrir o arquivo" << std::endl;
        return;
    }

    int num_nodes;
    infile >> num_nodes;

    nodes.clear();
    nodes[0] = 0;

    for (int i = 1; i < num_nodes; ++i)
    { // Start from 1 because the first node is the origin
        int id, pedido;
        infile >> id >> pedido;
        nodes[id] = pedido;
    }

    int num_edges;
    infile >> num_edges;

    distances.clear();
    int node1, node2, distance;
    for (int i = 0; i < num_edges; ++i)
    {
        infile >> node1 >> node2 >> distance;
        distances[{node1, node2}] = distance;
    }

    infile.close();
}

/**
 * @brief Generates all permutations of node indices.
 *
 * This function generates all possible permutations of node indices based on the input locations.
 * It uses the standard library's next_permutation to produce all possible orderings of nodes.
 *
 * @param locations A map of node IDs and associated values.
 * @return A vector of vectors, where each inner vector represents a permutation of node indices.
 */

std::vector<std::vector<int>> generatePermutations(const std::map<int, int> &locations)
{
    std::vector<std::vector<int>> permutations;
    std::vector<int> indexes;

    for (const auto &pair : locations)
    {
        indexes.push_back(pair.first);
    }

    int n = indexes.size();
    int num_permutations = 1;
    for (int i = 1; i <= n; ++i)
    {
        num_permutations *= i;
    }

    for (int i = 0; i < num_permutations; ++i)
    {
        permutations.push_back(indexes);
        std::next_permutation(indexes.begin(), indexes.end());
    }

    return permutations;
}

/**
 * @brief Generates all permutations of node indices using parallel processing.
 *
 * This function generates all possible permutations of node indices using OpenMP for parallel processing.
 * Each permutation is calculated independently and stored in a shared vector.
 *
 * @param locations A map of node IDs and associated values.
 * @return A vector of vectors, where each inner vector represents a permutation of node indices, computed in parallel.
 */

std::vector<std::vector<int>> generatePermutationsParallel(const std::map<int, int> &locations)
{
    std::vector<std::vector<int>> permutations;
    std::vector<int> indexes;

    for (const auto &pair : locations)
    {
        indexes.push_back(pair.first);
    }

    int n = indexes.size();
    int num_permutations = 1;
    for (int i = 1; i <= n; ++i)
    {
        num_permutations *= i;
    }

    permutations.resize(num_permutations);

#pragma omp parallel for
    for (int i = 0; i < num_permutations; ++i)
    {
        std::vector<int> local_indexes = indexes;
        for (int j = 0; j < i; ++j)
        {
            std::next_permutation(local_indexes.begin(), local_indexes.end());
        }
        permutations[i] = local_indexes;
    }

    return permutations;
}

/**
 * @brief Generates all permutations of node indices using parallel processing with optimized sorting.
 *
 * This function first generates all possible permutations of node indices in a sorted order and then copies them into
 * a parallel vector using OpenMP for optimized performance.
 *
 * @param locations A map of node IDs and associated values.
 * @return A vector of vectors, where each inner vector represents a permutation of node indices, optimized and computed in parallel.
 */

std::vector<std::vector<int>> generatePermutationsParallelOptimized(const std::map<int, int> &locations)
{
    std::vector<std::vector<int>> permutations;
    std::vector<int> indexes;

    for (const auto &pair : locations)
    {
        indexes.push_back(pair.first);
    }

    std::sort(indexes.begin(), indexes.end());

    do
    {
        permutations.push_back(indexes);
    } while (std::next_permutation(indexes.begin(), indexes.end()));

    int num_permutations = permutations.size();
    std::vector<std::vector<int>> parallel_permutations(num_permutations);

#pragma omp parallel for
    for (int i = 0; i < num_permutations; ++i)
    {
        parallel_permutations[i] = permutations[i];
    }

    return parallel_permutations;
}

/**
 * @brief Prints all permutations of nodes.
 *
 * This function prints each permutation of nodes to the standard output.
 *
 * @param permutations A vector of vectors, where each inner vector represents a permutation of node indices.
 */

void printPermutations(const std::vector<std::vector<int>> &permutations)
{
    std::cout << "Permutações:" << std::endl;

    for (const auto &permutation : permutations)
    {
        for (int node : permutation)
        {
            std::cout << node << " ";
        }
        std::cout << std::endl;
    }
}

/**
 * @brief Prints all possible paths.
 *
 * This function prints each possible path to the standard output.
 *
 * @param possiblePaths A vector of vectors, where each inner vector represents a possible path.
 */

void printPaths(const std::vector<std::vector<int>> &possiblePaths)
{
    std::cout << "Caminhos possíveis:" << std::endl;
    for (const auto &path : possiblePaths)
    {
        for (int distance : path)
        {
            std::cout << distance << " ";
        }
        std::cout << std::endl;
    }
}

/**
 * @brief Prints a single path with its associated cost.
 *
 * This function prints a path to the standard output along with a descriptive text and the total cost.
 *
 * @param path A vector representing the nodes in the path.
 * @param text A string description to accompany the path output.
 * @param cost The total cost associated with the path.
 */

void printPath(const std::vector<int> &path, const std::string &text, int cost)
{
    std::cout << text << ": ";
    for (int node : path)
    {
        std::cout << node << " ";
    }
    std::cout << " with cost: " << cost << std::endl;
}

/**
 * @brief Generates possible paths based on permutations, distances, and node capacities.
 *
 * This function takes permutations of nodes and generates paths that respect maximum capacity constraints.
 * It ensures that each path starts and ends at the origin node and that no path exceeds the maximum capacity.
 *
 * @param permutations A vector of vectors, where each inner vector represents a permutation of node indices.
 * @param distances A map of distances between pairs of nodes.
 * @param nodes A map of nodes and their capacities.
 * @param maxCapacity The maximum capacity constraint for each path.
 * @return A vector of vectors, where each inner vector represents a valid path respecting the capacity constraint.
 */

std::vector<std::vector<int>> generatePossiblePaths(std::vector<std::vector<int>> permutations, const std::map<std::pair<int, int>, int> &distances, const std::map<int, int> &nodes, int maxCapacity)
{
    std::vector<std::vector<int>> possiblePaths;
    int numPermutations = permutations.size();

    for (int i = 0; i < numPermutations; ++i)
    {
        std::vector<int> path;
        int capacity = 0;

        if (permutations[i][0] != 0)
        {
            permutations[i].insert(permutations[i].begin(), 0);
        }

        int permutationSize = permutations[i].size();

        for (int j = 0; j < permutationSize - 1; ++j)
        {
            int from = permutations[i][j];
            int to = permutations[i][j + 1];
            int nextNodeCapacity = nodes.at(to);

            auto it = distances.find({from, to});

            if (it != distances.end() && capacity + nextNodeCapacity <= maxCapacity)
            {
                path.push_back(from);
                capacity += nextNodeCapacity;
            }
            else
            {
                path.push_back(from);
                if (from != 0)
                {
                    path.push_back(0);
                }
                capacity = nextNodeCapacity;
            }
        }

        path.push_back(permutations[i].back());

        if (path.back() != 0)
        {
            path.push_back(0);
        }

        possiblePaths.push_back(path);
    }

    return possiblePaths;
}

/**
 * @brief Generates possible paths using parallel processing.
 *
 * This function takes permutations of nodes and generates paths that respect maximum capacity constraints in parallel.
 * It ensures that each path starts and ends at the origin node and that no path exceeds the maximum capacity.
 *
 * @param permutations A vector of vectors, where each inner vector represents a permutation of node indices.
 * @param distances A map of distances between pairs of nodes.
 * @param nodes A map of nodes and their capacities.
 * @param maxCapacity The maximum capacity constraint for each path.
 * @return A vector of vectors, where each inner vector represents a valid path respecting the capacity constraint, computed in parallel.
 */

std::vector<std::vector<int>> generatePossiblePathsParallel(const std::vector<std::vector<int>> &permutations, const std::map<std::pair<int, int>, int> &distances, const std::map<int, int> &nodes, int maxCapacity)
{
    std::vector<std::vector<int>> possiblePaths;
    int numPermutations = permutations.size();

    // Redimensiona o vetor de caminhos possíveis para que possa ser acessado em paralelo
    possiblePaths.resize(numPermutations);

#pragma omp parallel for
    for (int i = 0; i < numPermutations; ++i)
    {
        std::vector<int> path;
        int capacity = 0;

        std::vector<int> perm = permutations[i];

        if (perm[0] != 0)
        {
            perm.insert(perm.begin(), 0);
        }

        int permutationSize = perm.size();

        for (int j = 0; j < permutationSize - 1; ++j)
        {
            int from = perm[j];
            int to = perm[j + 1];
            int nextNodeCapacity = nodes.at(to);

            auto it = distances.find({from, to});

            if (it != distances.end() && capacity + nextNodeCapacity <= maxCapacity)
            {
                path.push_back(from);
                capacity += nextNodeCapacity;
            }
            else
            {
                path.push_back(from);
                if (from != 0)
                {
                    path.push_back(0);
                }
                capacity = nextNodeCapacity;
            }
        }

        path.push_back(perm.back());

        if (path.back() != 0)
        {
            path.push_back(0);
        }

        possiblePaths[i] = path;
    }

    return possiblePaths;
}

/**
 * @brief Finds the closest unvisited node from a given node.
 *
 * This function calculates the closest node to a given node from a set of unvisited nodes,
 * based on the distances between them. It returns the node ID of the closest node.
 *
 * @param node The current node ID.
 * @param unvisitedNodes A set of node IDs representing unvisited nodes.
 * @param nodes A map of nodes and their capacities.
 * @param distances A map of distances between pairs of nodes.
 * @return The node ID of the closest unvisited node. If no such node exists, returns -1.
 */

int findClosestNode(int node, const std::set<int> &unvisitedNodes, const std::map<int, int> &nodes, const std::map<std::pair<int, int>, int> &distances)
{
    int closestNode = -1;
    int minDistance = INT_MAX;

    std::vector<int> unvisitedVector(unvisitedNodes.begin(), unvisitedNodes.end());

#pragma omp parallel for shared(node, unvisitedVector, distances) reduction(min : minDistance)
    for (size_t i = 0; i < unvisitedVector.size(); ++i)
    {
        int candidate = unvisitedVector[i];
        if (candidate != node)
        {
            auto distIt = distances.find({node, candidate});
            if (distIt != distances.end())
            {
                int distance = distIt->second;
#pragma omp critical
                {
                    if (distance < minDistance)
                    {
                        minDistance = distance;
                        closestNode = candidate;
                    }
                }
            }
        }
    }

    return closestNode;
}
