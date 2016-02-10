#ifndef VNE_SOLUTION_BUILDER_H_
#define VNE_SOLUTION_BUILDER_H_

#include "cplex_solver.h"

class VNESolutionBuilder {
 public:
  VNESolutionBuilder(VNEReallocationCPLEXSolver *vne_solver_ptr,
                     Graph *physical_topology, 
                     std::vector<std::unique_ptr<Graph>> *virt_topologies)
      : vne_solver_ptr_(vne_solver_ptr),
        physical_topology_(physical_topology),
        virt_topologies_(virt_topologies) {}

  void PrintEdgeMappings(const char *vnr_directory);
  void PrintNodeMappings(const char *vnr_directory);
  void PrintCost(const char *filename);

 private:
  VNEReallocationCPLEXSolver *vne_solver_ptr_;
  Graph *physical_topology_;
  std::vector<std::unique_ptr<Graph>> *virt_topologies_;
};

#endif  // VNE_SOLUTION_BUILDER_H_
