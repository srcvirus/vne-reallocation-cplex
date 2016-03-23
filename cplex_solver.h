#ifndef CPLEX_SOLVER_
#define CPLEX_SOLVER_

#include "datastructure.h"
#include "util.h"

// #include <ilcplex/ilopcplex.h>
#include "ilcplex/ilocplex.h"

typedef IloArray<IloIntVarArray> IloIntVar2dArray;
typedef IloArray<IloIntVar2dArray> IloIntVar3dArray;
typedef IloArray<IloIntVar3dArray> IloIntVar4dArray;
typedef IloArray<IloIntVar4dArray> IloIntVar5dArray;
typedef IloArray<IloIntVar5dArray> IloIntVar6dArray;

typedef IloArray<IloIntArray> IloInt2dArray;
typedef IloArray<IloInt2dArray> IloInt3dArray;
typedef IloArray<IloInt3dArray> IloInt4dArray;
typedef IloArray<IloInt4dArray> IloInt5dArray;

typedef IloArray<IloExprArray> IloExpr2dArray;
using std::string;

class VNEReallocationCPLEXSolver {
 public:
  VNEReallocationCPLEXSolver() {}
  VNEReallocationCPLEXSolver(
      const std::unique_ptr<Graph>& physical_topology,
      const std::vector<std::unique_ptr<Graph>>& virt_topologies,
      const std::vector<std::unique_ptr<std::vector<std::vector<int>>>>&
          location_constraints,
      const std::vector<std::unique_ptr<VNEmbedding>>& vn_embeddings,
      const std::unique_ptr<VNRParameters>& vnr_parameters);
  void BuildModel();
  bool Solve();
  IloCplex& cplex() { return cplex_; }
  IloIntVar5dArray& X_imn_uv() { return X_imn_uv_; }
  IloIntVar3dArray& Y_im_u() { return Y_im_u_; }
  IloIntVar2dArray& is_bottleneck_u_v() { return is_bottleneck_u_v_; }
  IloIntVar6dArray& L_imn_uv_w() { return L_imn_uv_w_; }
  int max_channels() { return max_channels_; }

  const VNRParameters* vnr_parameters() const { return vnr_parameters_; }

 private:
  IloEnv env_;
  IloModel model_;
  IloCplex cplex_;
  IloConstraintArray constraints_;
  IloNumArray preferences_;
  Graph* physical_topology_;
  std::vector<Graph*> virt_topologies_;
  std::vector<std::vector<std::vector<int>>*> location_constraints_;
  std::vector<VNEmbedding*> vn_embeddings_;
  VNRParameters* vnr_parameters_;
  unsigned int max_channels_;
  // Link mapping decision variable.
  IloIntVar5dArray X_imn_uv_;
  // Node mapping decision variable.
  IloIntVar3dArray Y_im_u_;
  // Decision variable for vlink wavelength assignment.
  IloIntVar4dArray L_imn_w_;
  // Decision variable for plink wavelength assignment.
  IloIntVar6dArray L_imn_uv_w_;
  // Indicator decision variable for bottleneck link.
  IloIntVar2dArray is_bottleneck_u_v_;

  // Variable indicating location constraint.
  IloInt3dArray l_im_u_;
  // Variable indicating existing node mapping.
  IloInt3dArray y_im_u_;
  // Variable indicating existing link mapping.
  IloInt5dArray x_imn_uv_;
  // Variable indicating channel availability.
  IloInt3dArray av_uv_w_;
  // Objective function.
  IloExpr objective_;
};

#endif  // CPLEX_SOLVER_
