#include "../code/base.h"
#include "gtest/gtest.h"
#include <fstream>
#include <map>
#include <string>

// Helper function to create a temporary graph file for testing
void create_test_graph_file(const std::string &filename) {
  std::ofstream outfile(filename);
  outfile << "4\n";      // Number of nodes
  outfile << "1 10\n";   // Node 1, Request 10
  outfile << "2 20\n";   // Node 2, Request 20
  outfile << "3 30\n";   // Node 3, Request 30
  outfile << "4\n";      // Number of edges
  outfile << "0 1 5\n";  // Edge from 0 to 1, distance 5
  outfile << "1 2 10\n"; // Edge from 1 to 2, distance 10
  outfile << "2 3 15\n"; // Edge from 2 to 3, distance 15
  outfile << "0 3 20\n"; // Edge from 0 to 3, distance 20
  outfile.close();
}

TEST(BaseTest, LoadGraphTest) {
  // Arrange
  std::map<int, int> nodes;
  std::map<std::pair<int, int>, int> distances;
  std::string test_filename = "test_graph.txt";

  // Create the test graph file
  create_test_graph_file(test_filename);

  // Act
  load_graph(test_filename, nodes, distances);

  // Assert
  EXPECT_EQ(nodes.size(), 4);
  EXPECT_EQ(distances.size(), 4);

  // Checking node values
  EXPECT_EQ(nodes[1], 10);
  EXPECT_EQ(nodes[2], 20);
  EXPECT_EQ(nodes[3], 30);

  // Checking distance values
  int distance_01 = distances[{0, 1}];
  EXPECT_EQ(distance_01, 5);

  int distance_12 = distances[{1, 2}];
  EXPECT_EQ(distance_12, 10);

  int distance_23 = distances[{2, 3}];
  EXPECT_EQ(distance_23, 15);

  int distance_03 = distances[{0, 3}];
  EXPECT_EQ(distance_03, 20);

  // Clean up the test file
  remove(test_filename.c_str());
}

// Test case for generatePermutations function
TEST(BaseTest, GeneratePermutationsTest) {
  // Arrange
  std::map<int, int> locations = {{1, 10}, {2, 20}, {3, 30}};

  // Act
  std::vector<std::vector<int>> permutations = generatePermutations(locations);

  // Assert: Check that the number of permutations is correct (n!)
  int expectedPermutationsCount = 6; // 3! = 6
  EXPECT_EQ(permutations.size(), expectedPermutationsCount);

  // Define expected permutations
  std::vector<int> expectedPerm1 = {1, 2, 3};
  std::vector<int> expectedPerm2 = {1, 3, 2};
  std::vector<int> expectedPerm3 = {2, 1, 3};
  std::vector<int> expectedPerm4 = {2, 3, 1};
  std::vector<int> expectedPerm5 = {3, 1, 2};
  std::vector<int> expectedPerm6 = {3, 2, 1};

  // Helper lambda to check if the permutation exists
  auto contains_permutation = [&](const std::vector<int> &expected_perm) {
    return std::any_of(
        permutations.begin(), permutations.end(),
        [&](const std::vector<int> &perm) { return perm == expected_perm; });
  };

  // Assert each expected permutation exists
  EXPECT_TRUE(contains_permutation(expectedPerm1));
  EXPECT_TRUE(contains_permutation(expectedPerm2));
  EXPECT_TRUE(contains_permutation(expectedPerm3));
  EXPECT_TRUE(contains_permutation(expectedPerm4));
  EXPECT_TRUE(contains_permutation(expectedPerm5));
  EXPECT_TRUE(contains_permutation(expectedPerm6));
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
