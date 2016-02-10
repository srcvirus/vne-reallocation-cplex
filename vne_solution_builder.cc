#include "vne_solution_builder.h"
#include "util.h"

#include <math.h>

void VNESolutionBuilder::PrintEdgeMappings(const char *vnr_directory) {
  auto cplex = vne_solver_ptr_->cplex();
  auto X_imn_uv = vne_solver_ptr_->X_imn_uv();
  for (int i = 0; i < virt_topologies_->size(); ++i) {
    std::string emap_file = vnr_directory;
    emap_file += "vn" + std::to_string(i) + ".edge_remap.txt";
    FILE* outfile = fopen(emap_file.c_str(), "w");
    for (int m = 0; m < virt_topologies_->at(i)->node_count(); ++m) {
      auto &m_neighbors = virt_topologies_->at(i)->adj_list()->at(m);
      for (auto vend_point : m_neighbors) {
        int n = vend_point.node_id;
        if (m < n) continue;
        for (int u = 0; u < physical_topology_->node_count(); ++u) {
          auto &u_neighbors = physical_topology_->adj_list()->at(u);
          for (auto &end_point : u_neighbors) {
            int v = end_point.node_id;
            if (fabs(cplex.getValue(X_imn_uv[i][m][n][u][v]) - 1) < EPS) {
              printf(
                  "VN %d: Virtual edge (%d, %d) --> physical edge (%d, %d)\n",
                  i, m, n, u, v);
              fprintf(outfile, "VN %d: Virtual edge (%d, %d) --> physical "
                               "edge (%d, %d)\n", i, m, n, u, v);
            }
          }
        }
      }
    }
    fclose(outfile);
  }
}

void VNESolutionBuilder::PrintNodeMappings(const char *vnr_directory) {
  auto &cplex = vne_solver_ptr_->cplex();
  auto &Y_im_u = vne_solver_ptr_->Y_im_u();
  for (int i = 0; i < virt_topologies_->size(); ++i) {
    std::string nmap_file = vnr_directory;
    nmap_file += "vn" + std::to_string(i) + ".node_remap.txt";
    FILE* outfile = fopen(nmap_file.c_str(), "w");
    for (int m = 0; m < virt_topologies_->at(i)->node_count(); ++m) {
      for (int u = 0; u < physical_topology_->node_count(); ++u) {
        if (fabs(cplex.getValue(Y_im_u[i][m][u]) - 1) < EPS) {
          printf("VN %d: Virtual node %d --> physical node %d\n", i, m, u);
          fprintf(outfile, "VN %d: Virtual node %d --> physical node %d\n", i, m, u);
        }
      }
    }
    fclose(outfile);
  }
}

void VNESolutionBuilder::PrintCost(const char *filename) {
  FILE *outfile = NULL;
  if (filename) outfile = fopen(filename, "w");
  auto &cplex = vne_solver_ptr_->cplex();
  printf("Cost = %lf\n", cplex.getObjValue());
  if (outfile) {
    fprintf(outfile, "Cost = %lf\n", cplex.getObjValue());
  }
}
