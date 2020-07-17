# load JuMP
using JuMP
using CPLEX


function build_model(pck_matrix, dlv_matrix)
    n = size(pck_matrix, 1)                         # get dimension of the matrix (number nodes)
    m = Model(CPLEX.Optimizer)                      # get a model with CPLEX as Optimizer

    # _____________________________________ VARIABLES _____________________________________
    # we have a variable x for every possible arc, 1 for pck, 2 for delivery
    @variables(m, begin
        x1[1:n, 1:n], (Bin)                      
        x2[1:n, 1:n], (Bin)
    end)                           # Bin for binary variables, n x n arcs

    # cost of the two trips
    @variables(m, begin
        cost_pck
        cost_dlv
    end)

    # _____________________________________ OBJECTIVE _____________________________________
    # Compute the objective as the sum of the two tour
    sum_trip = @expression(m, cost_pck + cost_dlv)
    @objective(m, Min, sum_trip)
    
    
    # _____________________________________ CONSTRAINT _____________________________________
    # Calculate the delta variables         v1 vertex 1, v2 vertex 2
    # @constraint(m, delta_pck-[v1 in 1:n], sum(x1[v2, v1] for v2 in 1:n) == delta-_pck[v1])
    # @constraint(m, delta_pck+[v1 in 1:n], sum(x1[v1, v2] for v2 in 1:n) == delta+_pck[v1])
    # @constraint(m, delta_dlv-[v1 in 1:n], sum(x2[v2, v1] for v2 in 1:n) == delta-_dlv[v1])
    # @constraint(m, delta_dlv+[v1 in 1:n], sum(x2[v1, v2] for v2 in 1:n) == delta+_dlv[v1])
    
    # constraint 1: outgoing archs = incoming archs from source: x_t(δ+(0)) = x_t(δ−(0)) t = 1, 2
    @constraint(m, c1_pck, sum(x1[1, v] for v in 1:n) == sum(x1[v, 1] for v in 1:n))
    @constraint(m, c1_dlv, sum(x2[1, v] for v in 1:n) == sum(x2[v, 1] for v in 1:n))

    # constraint 2: every vertex visited only once: x_t(δ+(v)) = 1 for v = 1, 2, ..., n, t = 1, 2
    # delta+ number of outgoing arcs:   δ+(S) = {(v, w) ∈ A: v ∈ S, w /∈ S}
    @constraint(m, c2_pck[v1 in 2:n], sum(x1[v1, v2] for v2 in 1:n) == 1)
    @constraint(m, c2_dlv[v1 in 2:n], sum(x2[v1, v2] for v2 in 1:n) == 1)

    # constraint 3: every vertex visited only once: x_t(δ-(v)) = 1 for v = 1, 2, ..., n, t = 1, 2
    # delta- number of incoming arcs:   δ-(S) = {(v, w) ∈ A: v /∈ S, w ∈ S}    
    @constraint(m, c3_pck[v2 in 2:n], sum(x1[v1, v2] for v1 in 1:n) == 1)
    @constraint(m, c3_dlv[v2 in 2:n], sum(x2[v1, v2] for v1 in 1:n) == 1)

    # calculate cost of the tour for pck and dlv
    @constraint(m, pckcost, sum(pck_matrix[i,j] * x1[i,j] for i in 1:n, j in 1:n) == cost_pck)
    @constraint(m, dlvcost, sum(dlv_matrix[i,j] * x2[i,j] for i in 1:n, j in 1:n) == cost_dlv)
    
    # no self loop
    @constraint(m, no_self_pck[i in 1:n], x1[i,i] == 0)
    @constraint(m, no_self_dlv[i in 1:n], x2[i,i] == 0)


    # Printing the prepared optimization model
    print(m)

    # Solving the optimization problem
    optimize!(m)
    println("Optimal tour solutions:")
    println("Pck tour = ", JuMP.value(cost_pck))
    println("Dlv tour = ", JuMP.value(cost_dlv))
    println("X1 tour = ", JuMP.value.(x1))
    println("X2 tour = ", JuMP.value.(x2))

    return m
end


function solve()

end