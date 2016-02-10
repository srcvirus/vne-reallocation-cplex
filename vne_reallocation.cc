#include "cplex_solver.h"
#include "io.h"
#include "util.h"
#include "vne_solution_builder.h"

#include <iostream>

const std::string kUsage =
    "./vne_reallocation "
    "--case_directory=<case_directory>";

int main(int argc, char* argv[]) {
  using std::string;
  auto arg_map = ParseArgs(argc, argv);
  string case_directory = "";
  for (auto argument : *arg_map) {
    if (argument.first == "--case_directory") {
      case_directory = argument.second;
    } else {
      printf("Invalid command line option: %s\n", argument.first.c_str());
      return 1;
    }
  }
  const string kPhysicalTopologyFile = case_directory + "/sn.txt";
  auto physical_topology =
      InitializeTopologyFromFile(kPhysicalTopologyFile.c_str());
  const string kInfoFileName = case_directory + "/info";
  FILE* info_file = fopen(kInfoFileName.c_str(), "r");
  char buf[64];
  while (fgets(buf, sizeof(buf), info_file) != NULL) {
    string buf_wrap = buf;
    if (!buf_wrap.compare(0, 3, "VN:")) break;
  }
  if (fgets(buf, sizeof(buf), info_file) == NULL) {
    printf("info file has invalid format\n");
    return 1;
  }
  fclose(info_file);
  int num_vns = 0;
  sscanf(buf, "number = %d", &num_vns);
  std::vector<std::unique_ptr<Graph>> virt_topologies;
  std::vector<std::unique_ptr<std::vector<std::vector<int>>>>
      location_constraints;
  std::vector<std::unique_ptr<VNEmbedding>> vn_embeddings;
  long previous_cost = 0;
  for (int i = 0; i < num_vns; ++i) {
    const string kVirtTopologyFile =
        case_directory + "/vnr/vn" + std::to_string(i) + ".txt";
    const string kVNLocationConstraintFile =
        case_directory + "/vnr/vnloc" + std::to_string(i) + ".txt";
    const string kVLinkEmbeddingFile = kVirtTopologyFile + ".semap";
    const string kVNodeEmbeddingFile = kVirtTopologyFile + ".nmap";
    virt_topologies.emplace_back(
        InitializeTopologyFromFile(kVirtTopologyFile.c_str()));
    DEBUG(virt_topologies[i]->GetDebugString().c_str());
    location_constraints.emplace_back(InitializeVNLocationsFromFile(
        kVNLocationConstraintFile.c_str(), virt_topologies[i]->node_count()));
    vn_embeddings.emplace_back(InitializeVNEmbeddingFromFile(
        kVNodeEmbeddingFile.c_str(), kVLinkEmbeddingFile.c_str()));
    previous_cost +=
        EmbeddingCost(physical_topology.get(), virt_topologies[i].get(),
                      vn_embeddings[i].get());
  }
  ComputePhysicalNetworkCapacity(physical_topology.get(),
      virt_topologies, vn_embeddings);
  auto vnr_parameters = InitializeParametersFromFile(
      (case_directory + "/optimize_para.txt").c_str());
  DEBUG("Num VNs = %d\n", num_vns);
  DEBUG(physical_topology->GetDebugString().c_str());

  std::unique_ptr<VNEReallocationCPLEXSolver> cplex_solver(
      new VNEReallocationCPLEXSolver(physical_topology, virt_topologies,
                                     location_constraints, vn_embeddings,
                                     vnr_parameters));
  try {
    cplex_solver->BuildModel();
    bool is_success = cplex_solver->Solve();
    auto& cplex = cplex_solver->cplex();
    if (!is_success) {
      std::cout << "Solution status: " << cplex.getStatus() << std::endl;
      std::cout << "X : " << cplex.getCplexStatus() << std::endl;
    } else {
      printf("Success\n");
      printf("Previous cost: %ld\n", previous_cost);
      std::unique_ptr<VNESolutionBuilder> solution_builder(
          new VNESolutionBuilder(cplex_solver.get(), physical_topology.get(),
                                 &virt_topologies));
      solution_builder->PrintCost(
          (case_directory + "/vnr/new_cost.txt").c_str());
      solution_builder->PrintNodeMappings((case_directory + "/vnr/").c_str());
      solution_builder->PrintEdgeMappings((case_directory + "/vnr/").c_str());
    }
  }
  catch (IloException& e) {
    printf("Exception thrown: %s\n", e.getMessage());
  }
  return 0;
}
