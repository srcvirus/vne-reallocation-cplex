#include "vne_solution_builder.h"
#include "util.h"

#include <math.h>

void VNESolutionBuilder::PrintEdgeMappings(const char *vnr_directory) {
  auto &cplex = vne_solver_ptr_->cplex();
  auto &X_imn_uv = vne_solver_ptr_->X_imn_uv();
  auto &L_imn_uv_w = vne_solver_ptr_->L_imn_uv_w();
  int max_channels = vne_solver_ptr_->max_channels();
  for (int i = 0; i < virt_topologies_->size(); ++i) {
    std::string emap_file = vnr_directory;
    emap_file += "vn" + std::to_string(i) + ".edge_remap.txt";
    FILE *outfile = fopen(emap_file.c_str(), "w");
    for (int m = 0; m < virt_topologies_->at(i)->node_count(); ++m) {
      auto &m_neighbors = virt_topologies_->at(i)->adj_list()->at(m);
      for (auto vend_point : m_neighbors) {
        int n = vend_point.node_id;
        if (m > n) continue;
        for (int u = 0; u < physical_topology_->node_count(); ++u) {
          auto &u_neighbors = physical_topology_->adj_list()->at(u);
          for (auto &end_point : u_neighbors) {
            int v = end_point.node_id;
            for (int w = 0; w < max_channels; ++w) {
              if (fabs(cplex.getValue(X_imn_uv[i][m][n][u][v]) - 1) < EPS &&
                  fabs(cplex.getValue(L_imn_uv_w[i][m][n][u][v][w]) - 1) <
                      EPS) {
                printf(
                    "VN %d: Virtual edge (%d, %d) --> physical edge (%d, %d): "
                    "channel %d\n",
                    i, m, n, u, v, w);
                fprintf(outfile,
                        "VN %d: Virtual edge (%d, %d) --> physical "
                        "edge (%d, %d): channel %d\n",
                        i, m, n, u, v, w);
              }
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
    FILE *outfile = fopen(nmap_file.c_str(), "w");
    for (int m = 0; m < virt_topologies_->at(i)->node_count(); ++m) {
      for (int u = 0; u < physical_topology_->node_count(); ++u) {
        if (fabs(cplex.getValue(Y_im_u[i][m][u]) - 1) < EPS) {
          printf("VN %d: Virtual node %d --> physical node %d\n", i, m, u);
          fprintf(outfile, "VN %d: Virtual node %d --> physical node %d\n", i,
                  m, u);
        }
      }
    }
    fclose(outfile);
  }
}

std::unique_ptr<VNEmbedding> VNESolutionBuilder::GenerateEmbedding(
    int vn_index) {
  std::unique_ptr<VNEmbedding> embedding(new VNEmbedding());
  embedding->node_map = std::unique_ptr<std::vector<int>>(
      new std::vector<int>(virt_topologies_->at(vn_index)->node_count()));
  embedding->edge_map = std::unique_ptr<std::map<
      std::pair<int, int>, std::pair<int, std::vector<std::pair<int, int>>>>>(
      new std::map<std::pair<int, int>,
                   std::pair<int, std::vector<std::pair<int, int>>>>());
  auto &cplex = vne_solver_ptr_->cplex();

  // Populate Node Mapping.
  auto &Y_im_u = vne_solver_ptr_->Y_im_u();
  for (int m = 0; m < virt_topologies_->at(vn_index)->node_count(); ++m) {
    for (int u = 0; u < physical_topology_->node_count(); ++u) {
      if (fabs(cplex.getValue(Y_im_u[vn_index][m][u]) - 1) < EPS) {
        embedding->node_map->at(m) = u;
        break;
      }
    }
  }

  // Populate Edge Mapping.
  auto &X_imn_uv = vne_solver_ptr_->X_imn_uv();
  auto &L_imn_uv_w = vne_solver_ptr_->L_imn_uv_w();
  int max_channels = vne_solver_ptr_->max_channels();
  for (int m = 0; m < virt_topologies_->at(vn_index)->node_count(); ++m) {
    auto &m_neighbors = virt_topologies_->at(vn_index)->adj_list()->at(m);
    for (auto vend_point : m_neighbors) {
      int n = vend_point.node_id;
      if (m > n) continue;
      for (int u = 0; u < physical_topology_->node_count(); ++u) {
        auto &u_neighbors = physical_topology_->adj_list()->at(u);
        for (auto &end_point : u_neighbors) {
          int v = end_point.node_id;
          for (int w = 0; w < max_channels; ++w) {
            if (fabs(cplex.getValue(X_imn_uv[vn_index][m][n][u][v]) - 1) <
                    EPS &&
                fabs(cplex.getValue(L_imn_uv_w[vn_index][m][n][u][v][w]) - 1) <
                    EPS) {
              if (embedding->edge_map->find(std::make_pair(m, n)) ==
                  embedding->edge_map->end()) {
                embedding->edge_map->insert(std::make_pair(
                    std::make_pair(m, n),
                    std::pair<int, std::vector<std::pair<int, int>>>()));
              }
              embedding->edge_map->find(std::make_pair(m, n))->second.first = w;
              embedding->edge_map->find(std::make_pair(m, n))
                  ->second.second.push_back(std::make_pair(u, v));
            }
          }
        }
      }
    }
  }
  return std::move(embedding);
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

void VNESolutionBuilder::PrintSolutionStatus(const char *filename) {
  auto &cplex = vne_solver_ptr_->cplex();
  std::cout << "Solution status: " << cplex.getStatus() << std::endl;
  if (filename) {
    std::ofstream ofs(filename);
    ofs << cplex.getStatus();
    ofs.close();
  }
}
