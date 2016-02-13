#include "cplex_solver.h"

#include <unistd.h>

VNEReallocationCPLEXSolver::VNEReallocationCPLEXSolver(
    const std::unique_ptr<Graph>& physical_topology,
    const std::vector<std::unique_ptr<Graph>>& virt_topologies,
    const std::vector<std::unique_ptr<std::vector<std::vector<int>>>>&
        location_constraints,
    const std::vector<std::unique_ptr<VNEmbedding>>& vn_embeddings,
    const std::unique_ptr<VNRParameters>& vnr_parameters) {
  this->physical_topology_ = physical_topology.get();
  for (auto& vt : virt_topologies) {
    this->virt_topologies_.push_back(vt.get());
  }
  for (auto& lc : location_constraints) {
    this->location_constraints_.push_back(lc.get());
  }
  for (auto& vne : vn_embeddings) {
    this->vn_embeddings_.push_back(vne.get());
  }
  this->vnr_parameters_ = vnr_parameters.get();

  model_ = IloModel(env_);
  cplex_ = IloCplex(model_);
  constraints_ = IloConstraintArray(env_);
  preferences_ = IloNumArray(env_);
  objective_ = IloExpr(env_);

  // Initialize X_imn_uv_;
  X_imn_uv_ = IloIntVar5dArray(env_, virt_topologies_.size());
  // Initialize Y_im_u_;
  Y_im_u_ = IloIntVar3dArray(env_, virt_topologies_.size());
  // Initialize l_im_u_;
  l_im_u_ = IloInt3dArray(env_, virt_topologies_.size());
  // Initialize x_imn_uv_;
  x_imn_uv_ = IloInt5dArray(env_, virt_topologies_.size());
  // Initialize y_im_u_;
  y_im_u_ = IloInt3dArray(env_, virt_topologies_.size());
  // Initialize is_bottleneck_u_v_;
  is_bottleneck_u_v_ = IloIntVar2dArray(env_, physical_topology->node_count());

  for (int u = 0; u < physical_topology_->node_count(); ++u) {
    is_bottleneck_u_v_[u] =
        IloIntVarArray(env_, physical_topology_->node_count(), 0, 1);
  }

  for (int i = 0; i < virt_topologies_.size(); ++i) {
    X_imn_uv_[i] = IloIntVar4dArray(env_, virt_topologies_[i]->node_count());
    x_imn_uv_[i] = IloInt4dArray(env_, virt_topologies_[i]->node_count());
    for (int m = 0; m < virt_topologies_[i]->node_count(); ++m) {
      X_imn_uv_[i][m] =
          IloIntVar3dArray(env_, virt_topologies_[i]->node_count());
      x_imn_uv_[i][m] = IloInt3dArray(env_, virt_topologies_[i]->node_count());
      for (int n = 0; n < virt_topologies_[i]->node_count(); ++n) {
        X_imn_uv_[i][m][n] =
            IloIntVar2dArray(env_, physical_topology_->node_count());
        x_imn_uv_[i][m][n] =
            IloInt2dArray(env_, physical_topology_->node_count());
        for (int u = 0; u < physical_topology_->node_count(); ++u) {
          X_imn_uv_[i][m][n][u] =
              IloIntVarArray(env_, physical_topology_->node_count(), 0, 1);
          x_imn_uv_[i][m][n][u] =
              IloIntArray(env_, physical_topology_->node_count(), 0, 1);
          for (int v = 0; v < physical_topology_->node_count(); ++v) {
            x_imn_uv_[i][m][n][u][v] = 0;
          }
        }
      }
    }
  }
  for (int i = 0; i < virt_topologies_.size(); ++i) {
    Y_im_u_[i] = IloIntVar2dArray(env_, virt_topologies_[i]->node_count());
    y_im_u_[i] = IloInt2dArray(env_, virt_topologies_[i]->node_count());
    l_im_u_[i] = IloInt2dArray(env_, virt_topologies_[i]->node_count());
    for (int m = 0; m < virt_topologies_[i]->node_count(); ++m) {
      Y_im_u_[i][m] =
          IloIntVarArray(env_, physical_topology_->node_count(), 0, 1);
      y_im_u_[i][m] = IloIntArray(env_, physical_topology_->node_count(), 0, 1);
      l_im_u_[i][m] = IloIntArray(env_, physical_topology_->node_count(), 0, 1);
      for (int u = 0; u < physical_topology_->node_count(); ++u) {
        y_im_u_[i][m][u] = 0;
        l_im_u_[i][m][u] = 0;
      }
    }
  }

  for (int i = 0; i < vn_embeddings_.size(); ++i) {
    auto& vne = *vn_embeddings[i];
    for (int m = 0; m < vne.node_map->size(); ++m) {
      int u = vne.node_map->at(m);
      y_im_u_[i][m][u] = 1;
    }
    auto it = vne.edge_map->begin();
    for (auto it = vne.edge_map->begin(); it != vne.edge_map->end(); ++it) {
      int m = it->first.first, n = it->first.second;
      for (auto& edge : it->second) {
        int u = edge.first, v = edge.second;
        x_imn_uv_[i][m][n][u][v] = 1;
      }
    }
    auto& lc = *location_constraints[i];
    for (int m = 0; m < lc.size(); ++m) {
      for (auto u : lc[m]) {
        l_im_u_[i][m][u] = 1;
      }
    }
  }
}

void VNEReallocationCPLEXSolver::BuildModel() {
  // Constraint: Location constraint of virtual nodes.
  for (int i = 0; i < virt_topologies_.size(); ++i) {
    for (int m = 0; m < virt_topologies_[i]->node_count(); ++m) {
      for (int u = 0; u < physical_topology_->node_count(); ++u) {
        constraints_.add(Y_im_u_[i][m][u] <= l_im_u_[i][m][u]);
      }
    }
  }

  // Constraint: A virtual node is mapped to exactly one physical node.
  for (int i = 0; i < virt_topologies_.size(); ++i) {
    for (int m = 0; m < virt_topologies_[i]->node_count(); ++m) {
      IloIntExpr sum(env_);
      for (int u = 0; u < physical_topology_->node_count(); ++u) {
        sum += Y_im_u_[i][m][u];
      }
      constraints_.add(sum == 1);
    }
  }

  // Constraint: A physical node does not host more than one virtual node from
  // the same virtual network.
  for (int i = 0; i < virt_topologies_.size(); ++i) {
    for (int u = 0; u < physical_topology_->node_count(); ++u) {
      IloIntExpr sum(env_);
      for (int m = 0; m < virt_topologies_[i]->node_count(); ++m) {
        sum += Y_im_u_[i][m][u];
      }
      constraints_.add(sum <= 1);
    }
  }

  // Constraint: every virtual link should be mapped.
  for (int i = 0; i < virt_topologies_.size(); ++i) {
    for (int m = 0; m < virt_topologies_[i]->node_count(); ++m) {
      auto& m_neighbors = virt_topologies_[i]->adj_list()->at(m);
      for (auto& vend_point : m_neighbors) {
        int n = vend_point.node_id;
        if (m > n) continue;
        IloIntExpr sum(env_);
        for (int u = 0; u < physical_topology_->node_count(); ++u) {
          auto& u_neighbors = physical_topology_->adj_list()->at(u);
          for (auto& end_point : u_neighbors) {
            int v = end_point.node_id;
            constraints_.add(IloIfThen(env_, X_imn_uv_[i][m][n][u][v] == 1,
                                       X_imn_uv_[i][m][n][v][u] == 0));
            constraints_.add(IloIfThen(env_, X_imn_uv_[i][m][n][v][u] == 1,
                                       X_imn_uv_[i][m][n][u][v] == 0));
            sum += X_imn_uv_[i][m][n][u][v];
          }
        }
        constraints_.add(sum >= 1);
      }
    }
  }

  // Constraint: No over-commitment of physical link bandwidth.
  for (int u = 0; u < physical_topology_->node_count(); ++u) {
    auto& u_neighbors = physical_topology_->adj_list()->at(u);
    for (auto& end_point : u_neighbors) {
      int v = end_point.node_id;
      int beta_uv = end_point.bandwidth;
      IloIntExpr sum(env_);
      for (int i = 0; i < virt_topologies_.size(); ++i) {
        for (int m = 0; m < virt_topologies_[i]->node_count(); ++m) {
          auto& m_neighbors = virt_topologies_[i]->adj_list()->at(m);
          for (auto& vend_point : m_neighbors) {
            int n = vend_point.node_id;
            if (m > n) continue;
            int beta_mn = vend_point.bandwidth;
            sum += ((X_imn_uv_[i][m][n][u][v] + X_imn_uv_[i][m][n][u][v]) *
                    beta_mn);
          }
        }
      }
      constraints_.add(sum <= beta_uv);
      constraints_.add(is_bottleneck_u_v_[u][v] - is_bottleneck_u_v_[v][u] == 0);
      IloInt threshold = (vnr_parameters_->util_threshold * beta_uv);
      constraints_.add(
          IloIfThen(env_, sum > threshold, is_bottleneck_u_v_[u][v] == 1));
      constraints_.add(
          IloIfThen(env_, is_bottleneck_u_v_[u][v] == 1, sum > threshold));
      constraints_.add(
          IloIfThen(env_, sum <= threshold, is_bottleneck_u_v_[u][v] == 0));
      constraints_.add(
          IloIfThen(env_, is_bottleneck_u_v_[u][v] == 0, sum <= threshold));
    }
  }

  // Constraint: Flow constraint for path connectivity.
  for (int i = 0; i < virt_topologies_.size(); ++i) {
    for (int m = 0; m < virt_topologies_[i]->node_count(); ++m) {
      auto& m_neighbors = virt_topologies_[i]->adj_list()->at(m);
      for (auto& vend_point : m_neighbors) {
        int n = vend_point.node_id;
        if (m > n) continue;
        for (int u = 0; u < physical_topology_->node_count(); ++u) {
          IloIntExpr sum(env_);
          auto& u_neighbors = physical_topology_->adj_list()->at(u);
          for (auto& end_point : u_neighbors) {
            int v = end_point.node_id;
            sum += (X_imn_uv_[i][m][n][u][v] - X_imn_uv_[i][m][n][v][u]);
          }
          constraints_.add(sum == (Y_im_u_[i][m][u] - Y_im_u_[i][n][u]));
        }
      }
    }
  }

  // Objective function.
  // Part-1: Bandwidth cost.
  for (int i = 0; i < virt_topologies_.size(); ++i) {
    for (int m = 0; m < virt_topologies_[i]->node_count(); ++m) {
      auto& m_neighbors = virt_topologies_[i]->adj_list()->at(m);
      for (auto& vend_point : m_neighbors) {
        int n = vend_point.node_id;
        if (m > n) continue;
        long beta_mn = vend_point.bandwidth;
        for (int u = 0; u < physical_topology_->node_count(); ++u) {
          auto& u_neighbors = physical_topology_->adj_list()->at(u);
          for (auto& end_point : u_neighbors) {
            int v = end_point.node_id;
            int cost_uv = end_point.cost;
            objective_ += (vnr_parameters_->alpha) *
                          (X_imn_uv_[i][m][n][u][v] * cost_uv * beta_mn);
          }
        }
      }
    }
  }

  // Part-2: Number of bottleneck links.
  for (int u = 0; u < physical_topology_->node_count(); ++u) {
    auto& u_neighbors = physical_topology_->adj_list()->at(u);
    for (auto& end_point : u_neighbors) {
      int v = end_point.node_id;
      if (u > v) continue;
      objective_ += (vnr_parameters_->beta * is_bottleneck_u_v_[u][v]);
    }
  }
  constraints_.add(objective_ >= 0.0);
  model_.add(constraints_);
  model_.add(IloMinimize(env_, objective_));
}

bool VNEReallocationCPLEXSolver::Solve() {
  bool is_success;
  int n_threads = sysconf(_SC_NPROCESSORS_ONLN) * 2;
  cplex_.setParam(IloCplex::Threads, n_threads);
  cplex_.setParam(IloCplex::PreDual, true);
  is_success = cplex_.solve();
  return is_success;
}
