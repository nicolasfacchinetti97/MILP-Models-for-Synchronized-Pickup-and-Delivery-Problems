# load JuMP
using JuMP
using CPLEX

m = Model(CPLEX.Optimizer)

@variable(m, x, Bin)                # Bin for binary variables
