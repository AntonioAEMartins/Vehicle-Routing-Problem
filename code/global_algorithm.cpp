#include <iostream>
#include <fstream>
#include <sstream>
#include <map>
#include <vector>

using namespace std;

void load_graph(const string &filename, map<int, int> &nodes, map<pair<int, int>, int> &distances)
{
    ifstream infile(filename);
    if (!infile)
    {
        cerr << "Erro ao abrir o arquivo" << endl;
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

vector<vector<int>> generatePermutations(const map<int, int> &locations)
{
    vector<vector<int>> permutations;
    vector<int> indexes;

    // Extrair as chaves do mapa e armazenar no vetor indexes
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
        next_permutation(indexes.begin(), indexes.end());
    }

    return permutations;
}

vector<vector<int>> generatePossiblePaths(vector<vector<int>> permutations, map<pair<int, int>, int> distances, map<int, int> &nodes, int maxCapacity)
{
    cout << "Max Capacity: " << maxCapacity << endl;
    vector<vector<int>> possiblePaths;
    int numPermutations = permutations.size();
    bool isPathPossible = true;

    for (int i = 0; i < numPermutations; ++i)
    {
        vector<int> path;
        int capacity = 0;

        if (permutations[i][0] != 0)
        {
            permutations[i].insert(permutations[i].begin(), 0);
        }

        int permutationSize = permutations[i].size();

        for (int j = 0; j < permutationSize; ++j)
        {
            int from = permutations[i][j];

            if (j + 1 >= permutationSize)
            {
                path.push_back(from);
                capacity = 0;
                break;
            }

            int to = permutations[i][j + 1];
            auto it = distances.find({from, to});

            if (it == distances.end())
            {
                path.push_back(from);
                path.push_back(0);
                capacity = nodes.at(to);
                // cout << "Capacidade: " << capacity << " Node Cap: " << nodes.at(to) << " | " << from << " -> " << to << endl;
            }
            else
            {
                path.push_back(from);
                capacity += nodes.at(to);
            }

            if (capacity > maxCapacity)
            {
                isPathPossible = false;
                break;
            }
        }
        // cout << "Outra Permutação" << endl;

        if (path[path.size() - 1] != 0)
        {
            path.push_back(0);
        }

        if (isPathPossible)
        {
            cout << "Path Possible" << endl;
            possiblePaths.push_back(path);
        }
    }

    return possiblePaths;
}

vector<int> findBestPath(vector<vector<int>> possiblePaths, map<pair<int, int>, int> &distances)
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

    cout << "Best Cost: " << minCost << endl;

    return bestPath;
}

void printPermutations(vector<vector<int>> permutations)
{
    cout << "Permutações:" << endl;

    for (const auto &permutation : permutations)
    {
        for (int node : permutation)
        {
            cout << node << " ";
        }
        cout << endl;
    }
}

void printPaths(vector<vector<int>> possiblePaths)
{
    cout << "Caminhos possíveis:" << endl;
    for (const auto &path : possiblePaths)
    {
        for (int distance : path)
        {
            cout << distance << " ";
        }
        cout << endl;
    }
}

int main()
{
    // Adicionar Capacidade do Veículo
    map<int, int> nodes;
    map<pair<int, int>, int> distances;
    load_graph("../grafo.txt", nodes, distances);

    for (const auto &edge : distances)
    {
        cout << edge.first.first << " -> " << edge.first.second << ": " << edge.second << endl;
    }

    vector<vector<int>> permutations = generatePermutations(nodes);

    vector<vector<int>> possiblePaths = generatePossiblePaths(permutations, distances, nodes, 11);

    vector<int> bestPath = findBestPath(possiblePaths, distances);

    cout << "Best Path:" << endl;

    for (int i : bestPath)
    {
        cout << i << " ";
    }

    return 0;
}

// 40 + 40 + 37 + 70 + 64 + 46 + 46 + 46 = 389
// 35 + 64 + 46 + 37 + 37 + 46 + 46 + 40 + 40 = 391