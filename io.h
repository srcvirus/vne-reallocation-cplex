#ifndef IO_H_
#define IO_H_

#include "datastructure.h"
#include "util.h"

#include <map>
#include <memory>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <string.h>

std::unique_ptr<std::map<std::string, std::string>> ParseArgs(int argc,
                                                              char *argv[]) {
  std::unique_ptr<std::map<std::string, std::string>> arg_map(
      new std::map<std::string, std::string>());
  for (int i = 1; i < argc; ++i) {
    char *key = strtok(argv[i], "=");
    char *value = strtok(NULL, "=");
    DEBUG(" [%s] => [%s]\n", key, value);
    arg_map->insert(std::make_pair(key, value));
  }
  return std::move(arg_map);
}

std::unique_ptr<std::vector<std::vector<std::string>>> ReadCSVFile(
    const char *filename) {
  DEBUG("[Parsing %s]\n", filename);
  FILE *file_ptr = fopen(filename, "r");
  if (!file_ptr) {
    DEBUG("Invalid file %s\n", filename);
    return NULL;
  }
  const static int kBufferSize = 1024;
  char line_buffer[kBufferSize];
  std::unique_ptr<std::vector<std::vector<std::string>>> ret_vector(
      new std::vector<std::vector<std::string>>());
  std::vector<std::string> current_line;
  int row_number = 0;
  while (fgets(line_buffer, kBufferSize, file_ptr)) {
    DEBUG("Read %d characters\n", strlen(line_buffer));
    if (strlen(line_buffer) <= 0) continue;
    if (line_buffer[0] == '\n' || line_buffer[0] == '\r') continue;
    current_line.clear();
    char *token = strtok(line_buffer, ",\n\r");
    current_line.push_back(token);
    while ((token = strtok(NULL, ",\n"))) {
      current_line.push_back(token);
    }
    ret_vector->push_back(current_line);
  }
  fclose(file_ptr);
  DEBUG("Parsed %d lines\n", static_cast<int>(ret_vector->size()));
  return std::move(ret_vector);
}

std::unique_ptr<Graph> InitializeTopologyFromFile(const char *filename) {
  int node_count = 0, edge_count = 0;
  auto csv_vector = ReadCSVFile(filename);
  if (csv_vector.get() == NULL) {
    return NULL;
  }
  std::unique_ptr<Graph> graph(new Graph());
  for (int i = 0; i < csv_vector->size(); ++i) {
    auto &row = csv_vector->at(i);

    // Each line has the following format:
    // LinkID, SourceID, DestinationID, PeerID, Cost, Bandwidth, Delay.
    int u = atoi(row[1].c_str());
    int v = atoi(row[2].c_str());
    int cost = atoi(row[4].c_str());
    long bw = atol(row[5].c_str());
    int delay = atoi(row[6].c_str());

    DEBUG("Line[%d]: u = %d, v = %d, cost = %d, bw = %ld, delay = %d\n", i, u,
          v, cost, bw, delay);
    graph->add_edge(u, v, bw, delay, cost);
  }
  return std::move(graph);
}

std::unique_ptr<std::vector<std::vector<int>>> InitializeVNLocationsFromFile(
    const char *filename, int num_virtual_nodes) {
  DEBUG("Parsing %s\n", filename);
  auto ret_vector = std::unique_ptr<std::vector<std::vector<int>>>(
      new std::vector<std::vector<int>>(num_virtual_nodes));
  auto csv_vector = ReadCSVFile(filename);
  if (csv_vector.get() == NULL) {
    return NULL;
  }
  for (int i = 0; i < csv_vector->size(); ++i) {
    auto &row = csv_vector->at(i);
    int vnode_id = atoi(row[0].c_str());
    for (int j = 1; j < row.size(); ++j) {
      ret_vector->at(vnode_id).push_back(atoi(row[j].c_str()));
    }
  }
  return std::move(ret_vector);
}

std::unique_ptr<VNEmbedding> InitializeVNEmbeddingFromFile(
    const char *nmap_file, const char *emap_file) {
  auto vn_embedding = std::unique_ptr<VNEmbedding>(new VNEmbedding());
  vn_embedding->node_map =
      std::unique_ptr<std::vector<int>>(new std::vector<int>());
  vn_embedding->edge_map = std::unique_ptr<
      std::map<std::pair<int, int>, std::vector<std::pair<int, int>>>>(
      new std::map<std::pair<int, int>, std::vector<std::pair<int, int>>>());
  FILE *nmap = fopen(nmap_file, "r");
  FILE *emap = fopen(emap_file, "r");
  char buf[256];
  while (fgets(buf, sizeof(buf), nmap) != NULL) {
    int vnode, vnode_map;
    sscanf(buf, "%d %d", &vnode, &vnode_map);
    if (vnode > static_cast<int>(vn_embedding->node_map->size()) - 1)
      vn_embedding->node_map->resize(vnode + 1);
    vn_embedding->node_map->at(vnode) = vnode_map;
  }
  fclose(nmap);
  while (fgets(buf, sizeof(buf), emap) != NULL) {
    int u, v, m, n;
    sscanf(buf, "%d %d %d %d", &u, &v, &m, &n);
    if (u > v) std::swap(u, v);
    if (m > n) std::swap(m, n);
    if (vn_embedding->edge_map->find(std::make_pair(m, n)) ==
        vn_embedding->edge_map->end()) {
      vn_embedding->edge_map->insert(std::make_pair(
          std::make_pair(m, n), std::vector<std::pair<int, int>>()));
    }
    vn_embedding->edge_map->find(std::make_pair(m, n))
        ->second.push_back(std::make_pair(u, v));
  }
  fclose(emap);
  return std::move(vn_embedding);
}

std::unique_ptr<VNRParameters> InitializeParametersFromFile(
    const char *parameter_file) {
  auto parameters = std::unique_ptr<VNRParameters>(new VNRParameters());
  FILE *param_file = fopen(parameter_file, "r");
  char buffer[256];
  const char *prefix[] = {"Goal Utilization", "alpha", "beta", "gamma"};
  enum {
    UTIL = 0,
    ALPHA,
    BETA,
    GAMMA
  };

  while (fgets(buffer, sizeof(buffer), param_file) != NULL) {
    for (int i = 0; i < 4; ++i) {
      if (strncmp(buffer, prefix[i], strlen(prefix[i])) == 0) {
        switch (i) {
          case UTIL:
            sscanf(buffer + strlen(prefix[i]) + 2, "%lf",
                   &parameters->util_threshold);
            parameters->util_threshold /= 100.0;
            break;
          case ALPHA:
            sscanf(buffer + strlen(prefix[i]) + 2, "%lf", &parameters->alpha);
            break;
          case BETA:
            sscanf(buffer + strlen(prefix[i]) + 2, "%lf", &parameters->beta);
            break;
          case GAMMA:
            sscanf(buffer + strlen(prefix[i]) + 2, "%lf", &parameters->gamma);
            break;
          default:
            DEBUG("Invalid parameter specified in %s\n", parameter_file);
        }
        break;
      }
    }
  }
  fclose(param_file);
  DEBUG("%s\n", parameters->GetDebugString().c_str());
  return std::move(parameters);
}
#endif  // IO_H_
