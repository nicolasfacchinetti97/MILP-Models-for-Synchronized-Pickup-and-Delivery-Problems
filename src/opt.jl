# load JuMP
using JuMP
using CPLEX
using DelimitedFiles

function build_model(pck_matrix, dlv_matrix, print_log, dump)
    """
    Build the base optimization problem with only 1 - 2 - 3 constraint

    Parameters
    ---------
    pck_matrix: matrix
        matrix of points distances of the pickup problem
    dlv_matrix: matrix
        matrix of points distances of the delivery problem
    print_log: boolean
        true if want the log of CPLEX, false otherwise
    dump: boolean
        true if want to dump the model to .lp file
    Return
    ---------
    Model
        the base MILP model of the problem
    """
    n = size(pck_matrix, 1)                         # get dimension of the matrix (number nodes)
    m = Model(CPLEX.Optimizer)                      # get a model with CPLEX as Optimizer
    set_optimizer_attribute(m, "CPX_PARAM_SCRIND", print_log)
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

    if dump
        JuMP.write_to_file(m, "init_dump.lp")
    end

    return m
end


function solve(model)
    """
    Solve the given model

    Parameters
    ---------
    model: Model
        MILP model of the problem    
    Return
    ---------
    pck_tour
        the cost of the pickup tour
    dlv_tour
        the cost of the delivery tour
    x1
        matrices of the choosen arches in the pickup tour
    x2
        matrices of the choosen arches in the delivery tour
    """
    # Solving the optimization problem
    optimize!(model)
    cost_pck = model[:cost_pck]
    cost_dlv = model[:cost_dlv]    
    x1 = model[:x1]
    x2 = model[:x2]
    if termination_status(model) == MOI.OPTIMAL
        pck_tour = value(cost_pck)
        dlv_tour = value(cost_dlv)
        x1 = value.(x1)
        x2 = value.(x2)
        return pck_tour, dlv_tour, x1, x2
    elseif termination_status(model) == MOI.TIME_LIMIT && has_values(model)
        pck_tour = value(cost_pck)
        dlv_tour = value(cost_dlv)
        x1 = value.(x1)
        x2 = value.(x2)
        return pck_tour, dlv_tour, x1, x2
    else
        error("The model was not solved correctly.")
    end  
end

function get_x1(model)
    """
    Query the model for the value of x1

    Parameters
    ---------
    model: Model
        MILP model of the probel
    Return
    ---------
    Array{Int64, 2}
        matrix x1
    """
    x1 = model[:x1]
    return value.(x1)
end

function get_x2(model)
    """
    Query the model for the value of x2

    Parameters
    ---------
    model: Model
        MILP model of the probel
    Return
    ---------
    Array{Int64, 2}
        matrix x2
    """
    x2 = model[:x2]
    return value.(x2)
end

function get_values(model)
    """
    return the pickup tour price, delivery tour price and the matrices x1 and x2

    Parameters
    ---------
    model: Model
        MILP model of the problem
    Return
    ---------
    tuple
        int
            pickup tour price
        int
            delivery tour price
        Array{Int64, 2}
            matrix of arches for the pickup problem
        Array{Int64, 2}
            matrix of arches for the delivery probel
    """
    cost_pck = model[:cost_pck]
    cost_dlv = model[:cost_dlv]    
    x1 = model[:x1]
    x2 = model[:x2]
    pck_tour = value(cost_pck)
    dlv_tour = value(cost_dlv)
    x1 = value.(x1)
    x2 = value.(x2)
    return pck_tour, dlv_tour, x1, x2
end

function add_dynamic_constraint(model, S, k, type)
    """
    Add dynamically the constraint (4) to a given model resricted to a set of nodes
    
    Parameters
    ---------
    model: Model
        MILP model of the problem
    S: Array{Int64, 1}
        set of points that violate the constraint
    k: int
        the capacity of the veichle
    type: int
        1 for pickup, 2 for delivery
    Return
    ---------
    Model
        the given model with the added constraint
    """
    bound = ceil(length(S)/k)                           # round the division to the upper integer
    x1, x2, n = get_x1_x2_n(model)
    if type == 1
        @constraint(model, sum(x1[j,v] for v in 1:n, j in S if v ∉ S) >= bound)
    else
        @constraint(model, sum(x2[j,v] for v in 1:n, j in S if v ∉ S) >= bound)
    end
    return model  
end

function get_opt(model)
    """
    return the value of the best solution and the best lower bound

    Parameters
    ----------
    model: Model
        MILP model of the probel
    Return
    ----------
    Float64
        best value
    Float64
        lower boun
    """
    optimal_objective = objective_value(model)
    optimal_bound =  objective_bound(model)
    return optimal_objective, optimal_bound
end

function get_x1_x2_n(model)
	"get x1, x2 and number of nodes to add constraint to the model
	
	Parameters
	----------
	model: Model
		MILP model of the probel
	Return
	----------
	Float64
		pck matrix (x1)
	Float64
		dlv matrix (x2)
	int
		number of nodes in the instance
	"
	x1 = model[:x1]
    x2 = model[:x2]
    n = size(x1, 1)
	return x1, x2, n
end

function add_permutation_overlap_constraint(model)
    """
    Add the constraint 16 - 17 to the model
    
    Parameters
    ----------
    model:
        base MILP model of the probel
    Return
    ----------
    Model
        the model with the added constraints
    """
	x1, x2, n = get_x1_x2_n(model)
    @constraint(model, c16_1[v in 2:n, w in 2:n], 2x1[v,w] <= 2x2[v,w] + x2[v,1] + x2[1,w])
    @constraint(model, c16_2[v in 2:n, w in 2:n], 2x2[v,w] <= 2x1[v,w] + x1[v,1] + x1[1,w])
    @constraint(model, c17_1[v in 2:n, w in 2:n], 2x1[v,1] + 2x1[1,w] <= 2 + 2x2[v,w] + x2[v,1] + x2[1,w])
	@constraint(model, c17_2[v in 2:n, w in 2:n], 2x2[v,1] + 2x2[1,w] <= 2 + 2x1[v,w] + x1[v,1] + x1[1,w])
	return model
end