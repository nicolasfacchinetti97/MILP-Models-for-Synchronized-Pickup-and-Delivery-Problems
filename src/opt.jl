# load JuMP
using JuMP
using CPLEX

m = Model(CPLEX.Optimizer)


# we have a variabile x for every possible arc, and one for pck and one for delivery
@variable(m, x, Bin)                # Bin for binary variables
