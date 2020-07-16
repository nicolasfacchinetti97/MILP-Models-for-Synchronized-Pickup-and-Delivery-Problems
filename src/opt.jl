# load JuMP
using JuMP
using CPLEX


function build_model(pck_matrix, dlv_matrix)
    n = size(matrix, 1)                             # get dimension of the matrix (number nodes)
    m = Model(CPLEX.Optimizer)                      # get a model with CPLEX as Optimizer

    # _______________________ VARIABLES _________________________
    # we have a variable x for every possible arc, 1 for pck, 2 for delivery
    @variable(m, x1[n,n], Bin)                      # Bin for binary variables, n x n arcs
    @variable(m, x2[n,n], Bin)                
    

    # Delta variables for counting the arcs outgoing and incoming from a node
    # delta+ number of outgoing arcs:   δ+(S) = {(v, w) ∈ A: v ∈ S, w /∈ S}
    # delta- number of incoming arcs:   δ-(S) = {(v, w) ∈ A: v /∈ S, w ∈ S}
    @variable(m, delta-_pck[n])
    @variable(m, delta+_pck[n])
    @variable(m, delta-_dlv[n])
    @variable(m, delta+_dlv[n])

    # cost of the two trips
    @variable(m, cost_pck)
    @variable(m, cost_dlv)

    # _______________________ OBJECTIVE _________________________
    # Setting the objective
    sum_trip = pck_cost + dlv_cost
    @objective(m, Min, sum_trip)
    
    # _______________________ CONSTRAINT _________________________
    # Calculate the delta variables         v1 vertex 1, v2 vertex 2
    @constraint(m, delta_pck-[v1 in 1:n], sum(x1[v2, v1] for v2 in 1:n) == delta-_pck[v1])
    @constraint(m, delta_pck+[v1 in 1:n], sum(x1[v1, v2] for v2 in 1:n) == delta+_pck[v1])
    @constraint(m, delta_dlv-[v1 in 1:n], sum(x2[v2, v1] for v2 in 1:n) == delta-_dlv[v1])
    @constraint(m, delta_dlv+[v1 in 1:n], sum(x2[v1, v2] for v2 in 1:n) == delta+_dlv[v1])
    
    # constraint 1: outgoing archs = incoming archs from source: x_t(δ+(0)) = x_t(δ−(0)) t = 1, 2
    @constraint(m, c1_pck, x1[] * delta+_pck[0] == x1 * delta-_pck[0])
    @constraint(m, c1_dlv, )

    # constraint 2: every vertex visited only once: x_t(δ+(v)) = 1 for v = 1, 2, . . . , n, t = 1, 2
    @constraint(m, c2_pck[v in i:n], x1[] * delta+_pck[v] == 1)
    @constraint(m, c2_dlv[v in i:n], x2[] * delta+_dlv[v] == 1)

    # constraint 3: every vertex visited only once: x_t(δ+(v)) = 1 for v = 1, 2, . . . , n, t = 1, 2
    @constraint(m, c3_pck[v in i:n], x1[] * delta-_pck[v] == 1)
    @constraint(m, c3_dlv[v in i:n], x2[] * delta-_dlv[v] == 1)

    # calculate cost of the tour for pck and dlv
    @constraint(m, pckcost, pck_matrix * x1 .== cost_pck)
    @constraint(m, dlvcost, dlv_matrix * x2 .== cost_dlv)
    
    return m
end


function solve()

end