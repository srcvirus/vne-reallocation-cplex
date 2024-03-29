LIB_PATHS = -L/opt/ibm/ILOG/CPLEX_Studio125/cplex/lib/x86-64_sles10_4.1/static_pic -L/opt/ibm/ILOG/CPLEX_Studio125/concert/lib/x86-64_sles10_4.1/static_pic

INCLUDE_PATHS = -I/opt/ibm/ILOG/CPLEX_Studio125/cplex/include -I/opt/ibm/ILOG/CPLEX_Studio125/concert/include
LIBS = -lilocplex -lconcert -lcplex -lm -lpthread  -DIL_STD

FILES = vne_reallocation.cc cplex_solver.cc util.cc vne_solution_builder.cc

all:
	g++ -O3 -std=c++0x $(LIB_PATHS) $(INCLUDE_PATHS) $(FILES) $(LIBS) -o vne_reallocation

dbg:
	g++ -DDBG -g -std=c++0x $(LIB_PATHS) $(INCLUDE_PATHS) $(FILES) $(LIBS) -o vne_reallocation

debug:
	g++ -g -std=c++0x $(LIB_PATHS) $(INCLUDE_PATHS) $(FILES) $(LIBS) -o vne_reallocation
