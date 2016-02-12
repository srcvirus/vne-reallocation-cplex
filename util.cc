#include "util.h"

#include <algorithm>
#include <map>
#include <stdarg.h>
#include <stdio.h>
#include <string>

void PrintDebugMessage(const char* location, const char* fmt_string, ...) {
  va_list args;
  va_start(args, fmt_string);
  std::string str = location;
  str += fmt_string;
  vprintf(str.c_str(), args);
  fflush(stdout);
  va_end(args);
}

template <class T>
double GetMean(const std::vector<T>& data) {
  T sum = T(0);
  const size_t kNumElements = data.size();
  for (auto& element : data) sum += element;
  return sum / static_cast<T>(kNumElements);
}

template <class T>
T GetNthPercentile(const std::vector<T>& data, int n) {
  std::vector<T> temp_data_buffer = data;
  sort(temp_data_buffer.begin(), temp_data_buffer.end());
  const size_t kNumElements = data.size();
  int rank = n * kNumElements;
  if (rank % 100) {
    rank = (rank / 100) + 1;
  } else
    rank /= 100;
  --rank;
  return temp_data_buffer[rank];
}

template <class T>
std::vector<std::pair<T, double>> GetCDF(const std::vector<T>& data) {
  int precision = 1;
  std::vector<T> temp_data_buffer = data;
  if (typeid(temp_data_buffer[0]) == typeid(double)||
      typeid(temp_data_buffer[0]) == typeid(float)) {
    precision = 1000;
  }
  std::map<int, int> cdf;
  for (int i = 0; i < temp_data_buffer.size(); ++i) {
    int bucket_index = temp_data_buffer[i] * precision;
    if (cdf[bucket_index])
      cdf[bucket_index]++;
    else
      cdf[bucket_index] = 1;
  }
  std::map<int, int>::iterator prev = cdf.begin(), current = cdf.begin();
  current++;
  for (; current != cdf.end(); current++, prev++) {
    current->second += prev->second;
  }
  int total = temp_data_buffer.size();
  std::vector<std::pair<T, double>> ret;
  for (current = cdf.begin(); current != cdf.end(); ++current) {
    T first = static_cast<T>(current->first) / static_cast<T>(precision);
    double second =
        static_cast<double>(current->second) / static_cast<double>(total);
    ret.push_back(std::make_pair(first, second));
  }
  return ret;
}

long BandwidthCost(
    const Graph* phys_topology,
    const std::vector<std::unique_ptr<Graph>>& virt_topologies,
    const std::vector<std::unique_ptr<VNEmbedding>>& vn_embeddings) {
  long cost = 0;
  for (int i = 0; i < virt_topologies.size(); ++i) {
    const VNEmbedding* embedding = vn_embeddings[i].get();
    const Graph* virt_topology = virt_topologies[i].get();
    for (auto emap_it = embedding->edge_map->begin();
         emap_it != embedding->edge_map->end(); ++emap_it) {
      auto& vlink = emap_it->first;
      auto& plinks = emap_it->second;
      for (auto& e : plinks) {
        cost += phys_topology->get_edge_cost(e.first, e.second) *
                virt_topology->get_edge_bandwidth(vlink.first, vlink.second);
      }
    }
  }
  return cost;
}

int GetNumBottleneckLinks(
    const Graph* phys_topology,
    const std::vector<std::unique_ptr<Graph>>& virt_topologies,
    const std::vector<std::unique_ptr<VNEmbedding>>& vn_embeddings,
    const VNRParameters* vnr_param) {
  int num_bottlenecks = 0;
  std::vector<std::vector<double>> util_matrix(
      phys_topology->node_count(),
      std::vector<double>(phys_topology->node_count(), 0.0));
  for (int i = 0; i < virt_topologies.size(); ++i) {
    const Graph* virt_topology = virt_topologies[i].get();
    const VNEmbedding* embedding = vn_embeddings[i].get();
    for (auto emap_it = embedding->edge_map->begin();
         emap_it != embedding->edge_map->end(); ++emap_it) {
      auto& vlink = emap_it->first;
      auto& plinks = emap_it->second;
      for (auto& e : plinks) {
        long b_mn =
            virt_topology->get_edge_bandwidth(vlink.first, vlink.second);
        util_matrix[e.first][e.second] += b_mn;
        util_matrix[e.second][e.first] += b_mn;
      }
    }
  }
  for (int u = 0; u < phys_topology->node_count(); ++u) {
    for (auto& end_point : phys_topology->adj_list()->at(u)) {
      int v = end_point.node_id;
      long b_uv = phys_topology->get_edge_bandwidth(u, v);
      util_matrix[u][v] /= static_cast<double>(b_uv);
      if (u < v && util_matrix[u][v] >= vnr_param->util_threshold) {
        ++num_bottlenecks;
      }
    }
  }
  return num_bottlenecks;
}

double VNRCost(const Graph* phys_topology,
               const std::vector<std::unique_ptr<Graph>>& virt_topologies,
               const std::vector<std::unique_ptr<VNEmbedding>>& vn_embeddings,
               const VNRParameters* vnr_param) {
  long bw_cost = BandwidthCost(phys_topology, virt_topologies, vn_embeddings);
  int num_bottlenecks = GetNumBottleneckLinks(phys_topology, virt_topologies,
                                              vn_embeddings, vnr_param);
  double cost = vnr_param->alpha * static_cast<double>(bw_cost) +
                vnr_param->beta * static_cast<double>(num_bottlenecks);
  return cost;
}

void ComputePhysicalNetworkCapacity(
    Graph* phys_topology,
    const std::vector<std::unique_ptr<Graph>>& virt_topologies,
    const std::vector<std::unique_ptr<VNEmbedding>>& vn_embeddings) {
  for (int i = 0; i < virt_topologies.size(); ++i) {
    const Graph* vn = virt_topologies[i].get();
    const VNEmbedding* vne = vn_embeddings[i].get();
    for (auto emap_it = vne->edge_map->begin(); emap_it != vne->edge_map->end();
         ++emap_it) {
      auto& vlink = emap_it->first;
      auto& plinks = emap_it->second;
      int m = vlink.first, n = vlink.second;
      long b_mn = vn->get_edge_bandwidth(m, n);
      for (auto plink_it = plinks.begin(); plink_it != plinks.end();
           ++plink_it) {
        int u = plink_it->first, v = plink_it->second;
        long cur_bw = phys_topology->get_edge_bandwidth(u, v);
        phys_topology->set_edge_bandwidth(u, v, cur_bw + b_mn);
        phys_topology->set_edge_bandwidth(v, u, cur_bw + b_mn);
      }
    }
  }
}
