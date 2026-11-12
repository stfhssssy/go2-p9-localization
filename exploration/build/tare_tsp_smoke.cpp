#include "tsp_solver/tsp_solver.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

int main() {
  tsp_solver_ns::DataModel data;
  data.distance_matrix = {{0, 10, 14, 10}, {10, 0, 10, 14},
                          {14, 10, 0, 10}, {10, 14, 10, 0}};
  tsp_solver_ns::TSPSolver solver(data);
  solver.Solve();
  std::vector<int> route;
  solver.getSolutionNodeIndex(route, false);
  auto sorted = route;
  std::sort(sorted.begin(), sorted.end());
  if (sorted != std::vector<int>{0, 1, 2, 3} ||
      route.front() != 0 || std::abs(solver.getPathLength() - 4.0) > 1e-9) {
    std::cerr << "TARE OR-Tools smoke failed\n";
    return 1;
  }
  std::cout << "{\"upstream_tsp_wrapper\":true,\"ortools\":\"9.8.3296\","
               "\"nodes\":4,\"tour_length_m\":4.0,\"passed\":true}\n";
  return 0;
}
