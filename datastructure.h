#ifndef DATASTRUCTURE_H_
#define DATASTRUCTURE_H_

#include <list>
#include <string>
#include <sstream>
#include <vector>
#include <math.h>
#include <map>
#include <memory>
#include <stdlib.h>

#define INF 99999999
#define MAXN 1000
#define NIL -1

// An entry in an adjacent list. An entry contains the node_id of the endpoint.
// The entry contains bandwidth, residual bandwidth, delay and cost of the
// corresponding edge.
struct edge_endpoint {
  int node_id;
  long bandwidth;
  long residual_bandwidth;
  int delay;
  int cost;
  edge_endpoint(int node_id, long bw, int delay, int cost)
      : node_id(node_id),
        bandwidth(bw),
        delay(delay),
        residual_bandwidth(bw),
        cost(cost) {}
  std::string GetDebugString() {
    return "ndoe_id = " + std::to_string(node_id) + ", bandwidth = " +
           std::to_string(bandwidth) + ", delay = " + std::to_string(delay) +
           ", cost = " + std::to_string(cost);
  }
};

class Graph {
 public:
  Graph() {
    adj_list_ = std::unique_ptr<std::vector<std::vector<edge_endpoint>>>(
        new std::vector<std::vector<edge_endpoint>>);
    node_count_ = edge_count_ = 0;
  }

  // Accessor methods.
  int node_count() const { return node_count_; }
  int edge_count() const { return edge_count_; }
  const std::vector<std::vector<edge_endpoint>>* adj_list() const {
    return static_cast<const std::vector<std::vector<edge_endpoint>>*>(
        adj_list_.get());
  }

  // u and v are 0-based identifiers of an edge endpoint. An edge is
  // bi-directional, i.e., calling Graph::add_edge with u = 1, v = 3 will add
  // both (1, 3) and (3, 1) in the graph.
  void add_edge(int u, int v, long bw, int delay, int cost) {
    if (adj_list_->size() < u + 1) adj_list_->resize(u + 1);
    if (adj_list_->size() < v + 1) adj_list_->resize(v + 1);
    for (int i = 0; i < adj_list_->at(u).size(); ++i) {
      if (adj_list_->at(u)[i].node_id == v) return;
    }
    adj_list_->at(u).push_back(edge_endpoint(v, bw, delay, cost));
    adj_list_->at(v).push_back(edge_endpoint(u, bw, delay, cost));
    ++edge_count_;
    node_count_ = adj_list_->size();
  }

  int get_edge_cost(int u, int v) const {
    auto& neighbors = adj_list_->at(u);
    for (auto& end_point : neighbors) {
      if (end_point.node_id == v) return end_point.cost;
    }
  }

  long get_edge_bandwidth(int u, int v) const {
    auto& neighbors = adj_list_->at(u);
    for (auto& end_point : neighbors) {
      if (end_point.node_id == v) return end_point.bandwidth;
    }
  }

  void set_edge_bandwidth(int u, int v, long bw) {
    auto& neighbors = adj_list_->at(u);
    for (auto& end_point : neighbors) {
      if (end_point.node_id == v) end_point.bandwidth = bw;
    }
  }

  std::string GetDebugString() {
    std::string ret_string = "node_count = " + std::to_string(node_count_);
    ret_string += ", edge_count = " + std::to_string(edge_count_) + "\n";
    for (int i = 0; i < node_count_; ++i) {
      auto& neighbors = adj_list_->at(i);
      ret_string += std::to_string(i) + " --> ";
      for (auto& neighbor : neighbors) {
        ret_string += " (" + neighbor.GetDebugString() + ")";
      }
      ret_string += "\n";
    }
    return ret_string;
  }

 private:
  std::unique_ptr<std::vector<std::vector<edge_endpoint>>> adj_list_;
  int node_count_, edge_count_;
};

struct VNEmbedding {
  std::unique_ptr<std::vector<int>> node_map;
  std::unique_ptr<
      std::map<std::pair<int, int>, std::vector<std::pair<int, int>>>> edge_map;
  long cost;
};

struct VNRParameters {
  // Weights of the cost components.
  double alpha, beta, gamma;
  // Threshold to determine bottleneck links.
  double util_threshold;
  VNRParameters() : alpha(0.5), beta(0.5), gamma(0.0), util_threshold(0.8) {}
  std::string GetDebugString() {
    return "alpha = " + std::to_string(alpha) + ", beta = " +
           std::to_string(beta) + ", gamma = " + std::to_string(gamma) +
           ", util_threshold = " + std::to_string(util_threshold);
  }
};
#endif  // MIDDLEBOX_PLACEMENT_SRC_DATASTRUCTURE_H_
