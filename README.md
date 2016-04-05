# CPLEX Implementation for Virtual Network Reallocation 
## Dependencies

The implementation uses IBM ILOG CPLEX C++ API (version 12.5 of CPLEX Studio).
We assume CPLEX is installed in /opt/ibm/ILOG/CPLEX_Studio125. In case CPLEX is
installed in a different directory replace /opt/ibm/ILOG/CPLEX_Studio125 in the
Makefile with the CPLEX installation directory.

## File Organization
  * io.h: Utility functions for I/O.
  * datastructure.h: Contains necessary data structure definitions.
  * util.h: Contains several utility functions.
  * cplex_solver.h(.cc) : Contains class representing the cplex solver.
  * vne_solution_builder.h(.cc) : Contains helper class for building the final.
    solution from cplex variables and writing the solutions to file.
  * vne_reallocation.cc: Contains the main function.
## How to run
```
$ make
$ ./vne_reallocation --case_directory=<case_directory>
```
Directory structure of each case directory is as follows:
```
case0
├── sn.txt
├── optimization_para.txt
└── vnr
    ├── vn0.txt
    ├── vn0.txt.nmap
    ├── vn0.txt.semap
    ├── vn1.txt
    ├── vn1.txt.nmap
    ├── vn1.txt.semap
    ├── vnloc0.txt
    └── vnloc1.txt
```

Description of these files are as follows:
  * sn.txt = Specification of a physical network
  * optimization_para.txt = Parameters for optimization 
  * vni.txt = Specification of the i-th virtual network request
  * vni.txt.nmap = The given node mapping of the i-th VN
  * vni.txt.semap = The given link mapping of the i-th VN
  * vnloci = Location constraint for the i-th VN
  
## Input file format

A topology file contains the list of edges. Each line contains a description of
an edge in a comma separated value (CSV) format. Format of a line is as follows:
```
<LinkID>,<SourceNodeId>,<DestinationNodeId>,<PeerID>,<Cost>,<Channels>,<Latency>[<Total Channels>]
```
Where,
  * LinkID = Id of a link. Ignored for both physical and virtual topology.
  * SourceNodeId = 0-based node index of the source of the link
  * DestinationNodeId = 0-based node index of the destination of the link
  * PeerID = Ignored
  * Cost = Cost of provisioning unit bandwidth on this link. Cost is ignored for
           virtual links.
  * Channels = In case of physical network, this is the residual number of
               channels remaining on that physical link. In case of virtual 
               network, this is the numebr of channels required for the virtual
               link (which is always set to 1). 
  * Delay = Latency of a physical link. In case of virtual link, this is the
            maximum delay requirement for the virtual link. (Not used)
  * Total Channels = This field is only present in physical network
                     specification. This represents the maximum number of 
                     channels supported by a physical link.

A location constraint file contains as many lines as the number of virtual
nodes. Each line is a comma separated list of values. The first value indicates
the id of a virtual node followed by the ids of physical nodes where this
virtual node can be mapped.

The node mapping file for a VN contains as many lines as the number of virtual
nodes in the corresponding virtual network. Each line has the following format:
```
<virtual_node_id> <mapped_physical_node_id>
```

The link mapping file contains multiple lines, where each line corresponds to a
physical link and a virtual link mapped onto that physical link. Each line is
formatted as follows:
```
<plink_endpoint_0> <plink_endpoint_1> <mapped_vlink_endpoint0> <mapped_vlink_endpoint_1> <peer> <channel_id>
```
`peer` is ignored and `channel_id` represents the index of the channel assigned
to that virtual link.

The optimization_para.txt file contains the following lines:
```
Goal Utilization = x%
alpha = <alpha>
beta = <beta>
```
Goal utilization is the utilization threshold for determining if a physical
link is bottleneck or not. alpha and beta are the weights of bandwidth cost and
bottleneck link cost in the objective function, respectively. 

Note: Nodes are numberded from `0 ... (n - 1)` in a network with `n` nodes.

## Output Files

Currently the solver prints output to the standard output and writes them to
the following output files insider `vnr` directory:

* prev_cost = Cost of embedding before reoptimization.
* prev_bnecks = Number of bottleneck links before reoptimization.
* prev_bw_cost = Bandwidth cost before reoptimization.
* prev_max_plink_util = Maximum physical link utilization before reoptimization.
* sol_time = Execution time (in seconds)
* new_cost = Cost of the reoptimized embedding according to the cost function.
* new_bnecks = Number of bottleneck links after reoptimization
* new_bw_cost = Bandwidth cost after reoptimization
* new_max_plink_util = Maximum plink utilization after reoptimization
* vn*.node_remap = node mapping of that VN in the reoptimized embedding
* vn*.edge_remap = edge mapping of that VN in the reoptimized embedding
