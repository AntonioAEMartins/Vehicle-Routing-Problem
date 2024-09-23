#include "gtest/gtest.h"
#include <cstdlib>
#include <fstream>

// Helper function to execute the global solution
bool execute_global_solution(const std::string &graph_file, std::string &output) {
  std::string compile_command = "c++ -std=c++20 -o execs/global_parallelization ../code/global_parallelization.cpp ../code/base.cpp";
  std::string executable = "./execs/global_parallelization";

  // Compile the executable
  int compile_result = std::system(compile_command.c_str());
  if (compile_result != 0) {
    return false;  // Compilation failed
  }

  // Execute the compiled executable with the graph file
  std::string command = executable + " " + graph_file + " > output.txt";
  int execution_result = std::system(command.c_str());
  if (execution_result != 0) {
    return false;  // Execution failed
  }

  // Read the output from the file
  std::ifstream output_file("output.txt");
  if (!output_file.is_open()) {
    return false;  // Output file not found
  }
  std::getline(output_file, output);
  output_file.close();

  // Clean up
  std::remove("output.txt");
  std::remove(executable.c_str());

  return true;
}

// Helper function to execute the global parallelization solution with OpenMP
bool execute_global_parallelization_solution_omp(const std::string &graph_file, std::string &output) {
  std::string compile_command = "clang++ -std=c++20 -fopenmp -o execs/global_parallelization ../code/global_parallelization.cpp ../code/base.cpp";
  std::string executable = "./execs/global_parallelization";

  // Compile the executable with OpenMP support
  int compile_result = std::system(compile_command.c_str());
  if (compile_result != 0) {
    return false;  // Compilation failed
  }

  // Execute the compiled executable with the graph file
  std::string command = executable + " " + graph_file + " > output.txt";
  int execution_result = std::system(command.c_str());
  if (execution_result != 0) {
    return false;  // Execution failed
  }

  // Read the output from the file
  std::ifstream output_file("output.txt");
  if (!output_file.is_open()) {
    return false;  // Output file not found
  }
  std::getline(output_file, output);
  output_file.close();

  // Clean up
  std::remove("output.txt");
  std::remove(executable.c_str());

  return true;
}

// Test case for the standard global solution
TEST(IntegrationTest, GlobalSolutionTest) {
  // Arrange
  std::string graph_file = "../grafo.txt";
  std::string expected_output = "672.0"; // Adjust based on actual expected output
  std::string actual_output;

  // Act
  bool execution_successful = execute_global_solution(graph_file, actual_output);

  // Assert
  ASSERT_TRUE(execution_successful) << "Compilation or execution failed!";
  EXPECT_EQ(actual_output, expected_output);
}

// Test case for OpenMP-enabled global parallelization
TEST(IntegrationTest, GlobalParallelizationOMPTest) {
  // Arrange
  std::string graph_file = "../grafo.txt";
  std::string expected_output = "672.0"; // Adjust based on actual expected output for parallelization
  std::string actual_output;

  // Act
  bool execution_successful = execute_global_parallelization_solution_omp(graph_file, actual_output);

  // Assert
  ASSERT_TRUE(execution_successful) << "Compilation or execution with OpenMP failed!";
  EXPECT_EQ(actual_output, expected_output);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
