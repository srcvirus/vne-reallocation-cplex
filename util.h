#ifndef UTIL_H_
#define UTIL_H_

#include "datastructure.h"

#include <vector>
#include <utility>

#define ONE_GIG 1000000000ULL
#define EPS 1e-8

#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)
#define AT __FILE__ ":" TOSTRING(__LINE__) " "

#ifdef DBG
#define DEBUG(...) PrintDebugMessage(AT, __VA_ARGS__)
#else
#define DEBUG(...)
#endif

// Prints a message formatted using fmt_string. Prepends the location of the
// file before the message.
void PrintDebugMessage(const char *location, const char *fmt_string, ...);

// Returns the mean of the vector of data.
template <class T>
double GetMean(const std::vector<T> &data);

// Returns the Nth percentile of the vector of data. Nth percentile is
// calculated as the (ceil(N / 100) * data.size() - 1)-th element from an array
// obtained by sorting data.
template <class T>
T GetNthPercentile(const std::vector<T> &data, int n);

// Returns the Cumalitive Distribution Frequence of the data items sotred in
// data. If the data items sotred in data are of type double, then a precision
// of 3 digits after the decimal points is used.
template <class T>
std::vector<std::pair<T, double> > GetCDF(const std::vector<T> &data);

// Returns the cost of embedding virt_topology on phys_topology.
long EmbeddingCost(const Graph *phys_topology, const Graph *virt_topology,
                   const VNEmbedding *embedding);

// If the input physical network contains residual bandwidth then compute the
// bandwidth capacity of the physical links by adding the bandwidths of embedded
// virtual links.
void ComputePhysicalNetworkCapacity(
    Graph *phys_topology, 
    const std::vector<std::unique_ptr<Graph>>& virt_topologies,
    const std::vector<std::unique_ptr<VNEmbedding>>& vn_embeddings);

#endif  // UTIL_H_
