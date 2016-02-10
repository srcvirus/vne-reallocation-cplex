# CPLEX Implementation for Virtual Network Reallocation for minimizing bandwidth allocation cost and bottlneck links, i.e., links with >80% utilization

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
Sample test cases are provided in data-set directory (case0 - case5). Directory
structure of each case directory is as follows:
```
case5
├── info
├── sn.txt
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
  * info = Generic information about the physical and virtual networks
  * sn.txt = Specification of a physical network
  * vni.txt = Specification of the i-th virtual network request
  * vni.txt.nmap = The given node mapping of the i-th VN
  * vni.txt.semap = The given link mapping of the i-th VN
  * vnloci = Location constraint for the i-th VN
  
## Input file format

The info file currently needs the following format (see provided file for
example):
```
VN:
number = <number of VNs>
```
In the provided info files, there can be other sections other than "VN:". For
the time being, the other sections are ignored.

A topology file contains the list of edges. Each line contains a description of
an edge in a comma separated value (CSV) format. Format of a line is as follows:
```
<LinkID>,<SourceNodeId>,<DestinationNodeId>,<PeerID>,<Cost>,<Bandwidth>,<Latency>
```
Where,
  * LinkID = Id of a link. Ignored for both physical and virtual topology.
  * SourceNodeId = 0-based node index of the source of the link
  * DestinationNodeId = 0-based node index of the destination of the link
  * PeerID = Ignored
  * Cost = Cost of provisioning unit bandwidth on this link. Cost is ignored for
           virtual links.
  * Bandwidth = Available bandwidth of a physical link. In case of virtual link,
                this is the bandwidth requirement
  * Delay = Latency of a physical link. In case of virtual link, this is the
            maximum delay requirement for the virtual link.

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
<plink_endpoint_0> <plink_endpoint_1> <mapped_vlink_endpoint0> <mapped_vlink_endpoint_1>
```

Please see the provided files as an example for better clarification.

*Nodes are numberded from `0 ... (n - 1)` in a network with `n` nodes.

## Output Files

Currently the solver prints output to the standard output and writes them to
the following output files as well:

* new_cost.txt = Cost of the reoptimized embedding according to the cost function.
* vn*.node_remap = node mapping of that VN in the reoptimized embedding
* vn*.edge_remap = edge mapping of that VN in the reoptimized embedding
