# load JuMP
using JuMP
using CPLEX
using DelimitedFiles


model = nothing

function build_model(pck_matrix, dlv_matrix, time, print_log)
    """
    Build the base optimization problem with only 1 - 2 - 3 constraint

    Parameters
    ---------
    pck_matrix: matrix
        matrix of points distances of the pickup problem
    dlv_matrix: matrix
        matrix of points distances of the delivery problem
    time: int
        sets the time limit (in seconds) of the solver
    print_log: boolean
        true if want the log of CPLEX, false otherwise
    Return
    ---------
    Model
        the base MILP model of the problem
    """
    n = size(pck_matrix, 1)                         # get dimension of the matrix (number nodes)
    global model = Model(CPLEX.Optimizer)                      # get a model with CPLEX as Optimizer
    set_time_limit_sec(model, time)
    set_optimizer_attribute(model, "CPX_PARAM_SCRIND", print_log)
    # _____________________________________ VARIABLES _____________________________________
    # we have a variable x for every possible arc, 1 for pck, 2 for delivery
    @variables(model, begin
        x1[1:n, 1:n], (Bin)                      
        x2[1:n, 1:n], (Bin)
    end)                           # Bin for binary variables, n x n arcs

    # cost of the two trips
    @variables(model, begin
        cost_pck
        cost_dlv
    end)

    # _____________________________________ OBJECTIVE _____________________________________
    # Compute the objective as the sum of the two tour
    sum_trip = @expression(model, cost_pck + cost_dlv)
    @objective(model, Min, sum_trip)
    
    # _____________________________________ CONSTRAINT _____________________________________
    # constraint 1: outgoing archs = incoming archs from source: x_t(δ+(0)) = x_t(δ−(0)) t = 1, 2
    @constraint(model, c1_pck, sum(x1[1, v] for v in 1:n) == sum(x1[v, 1] for v in 1:n))
    @constraint(model, c1_dlv, sum(x2[1, v] for v in 1:n) == sum(x2[v, 1] for v in 1:n))

    # constraint 2: every vertex visited only once: x_t(δ+(v)) = 1 for v = 1, 2, ..., n, t = 1, 2
    # delta+ number of outgoing arcs:   δ+(S) = {(v, w) ∈ A: v ∈ S, w /∈ S}
    @constraint(model, c2_pck[v1 in 2:n], sum(x1[v1, v2] for v2 in 1:n) == 1)
    @constraint(model, c2_dlv[v1 in 2:n], sum(x2[v1, v2] for v2 in 1:n) == 1)

    # constraint 3: every vertex visited only once: x_t(δ-(v)) = 1 for v = 1, 2, ..., n, t = 1, 2
    # delta- number of incoming arcs:   δ-(S) = {(v, w) ∈ A: v /∈ S, w ∈ S}    
    @constraint(model, c3_pck[v2 in 2:n], sum(x1[v1, v2] for v1 in 1:n) == 1)
    @constraint(model, c3_dlv[v2 in 2:n], sum(x2[v1, v2] for v1 in 1:n) == 1)

    # calculate cost of the tour for pck and dlv
    @constraint(model, pckcost, sum(pck_matrix[i,j] * x1[i,j] for i in 1:n, j in 1:n) == cost_pck)
    @constraint(model, dlvcost, sum(dlv_matrix[i,j] * x2[i,j] for i in 1:n, j in 1:n) == cost_dlv)
    
    # no self loop
    @constraint(model, no_self_pck[i in 1:n], x1[i,i] == 0)
    @constraint(model, no_self_dlv[i in 1:n], x2[i,i] == 0)

    # set the callback to check constraint 4
    MOI.set(model, MOI.LazyConstraintCallback(), callback_constraint4)

    return model
end

function get_model()
    """
    return the MILP model
    """
    return model
end

function callback_constraint4(cb_data)
    """
    callback to dynamic check the constraint 4

    Parameters
    ----------
    cb_data
        callback data
    """
    x1, x2, n = get_x1_x2_n(get_model())
    # need to add this strange syntax since cannot extract multiple variables from cb_data
    m_pck = callback_value.(Ref(cb_data), x1)
    m_dlv = callback_value.(Ref(cb_data), x2)
    k_pck = config.get_pck_k()
    k_dlv = config.get_dlv_k()

    println("\nChecking for violated constraints in pickup")
    if binary_array(m_pck)
        res_pck = check_constraint_4(model, m_pck, k_pck, 1)
        if res_pck != 0
            MOI.submit(model, MOI.LazyConstraint(cb_data), res_pck)
        end
    else
        println("\nNo integer solution for pickup, skipping...")
    end

    println("Checking for violated constraints in delivery")
    if binary_array(m_dlv)
        res_dlv = check_constraint_4(model, m_dlv, k_dlv, 2)
        if res_dlv != 0
            MOI.submit(model, MOI.LazyConstraint(cb_data), res_dlv)
        end
    else
        println("No integer solution for delivery, skipping...") 
    end    

end

function callback_constraint23(cb_data)
    """
    callback to dynamic check the constraint 23

    Parameters
    ----------
    cb_data
        callback data
    """
    # need to add this strange syntax since cannot extract multiple variables from cb_data
    x1, x2, n = get_x1_x2_n(get_model())
    m_dlv = callback_value.(Ref(cb_data), x2)
    y1, y2 = get_y1_y2(get_model())
    y1 = callback_value.(Ref(cb_data), y1)
    y2 = callback_value.(Ref(cb_data), y2)

    println("\nSearching the minimizing constraint 23")
    if (binary_array(m_dlv) && binary_array(y1) && binary_array(y2))
        res = check_constraint_23(model, m_dlv, y1, y2)
        if res != 0
            MOI.submit(model, MOI.LazyConstraint(cb_data), res)
        end
    else
        println("No integer solution for delivery matrix or y1 or y2, skipping...")
    end
end

function callback_c4_c23(cb_data)
    """
    callback to dynamic check the constraints 4 and 23

    Parameters
    ----------
    cb_data
        callback data
    """
    callback_constraint4(cb_data)
    callback_constraint23(cb_data)
end

function binary_array(m_arr)
    """
    checks whether an array has all its entries in 0/1

    Parameters
    ----------
    m_arr
        array to be checked 
    """
    result = true 
    for i in eachindex(m_arr)
        if m_arr[i] > 0.000001 && m_arr[i] < 0.999999
            result = false
        end
    end 
    return result
end


function solve(model, dump)
    """
    Solve the given model

    Parameters
    ---------
    model: Model
        MILP model of the problem
    dump: boolean
        dump the model or not   
    Return
    ---------
    int
        the cost of the pickup tour
    int
        the cost of the delivery tour
    Array{Float64, 2}
        matrices of the choosen arches in the pickup tour
    Array{Float64, 2}
        matrices of the choosen arches in the delivery tour
    Float64
        the solve time reported by the solver
    """
    # Solving the optimization problem
    optimize!(model)
    cost_pck = model[:cost_pck]
    cost_dlv = model[:cost_dlv]    
    x1 = model[:x1]
    x2 = model[:x2]
    if dump
        JuMP.write_to_file(model, "progress_dump.lp")
    end
    if termination_status(model) == MOI.OPTIMAL
        pck_tour = value(cost_pck)
        dlv_tour = value(cost_dlv)
        x1 = value.(x1)
        x2 = value.(x2)
        t = solve_time(model)
        return pck_tour, dlv_tour, x1, x2, t
    elseif termination_status(model) == MOI.TIME_LIMIT && has_values(model)
        println("\nMaximun search time elapsed, stopping...\n")
        pck_tour = value(cost_pck)
        dlv_tour = value(cost_dlv)
        x1 = value.(x1)
        x2 = value.(x2)
        t = solve_time(model)
        return pck_tour, dlv_tour, x1, x2, t
    else
        error("The model was not solved correctly.")
    end  
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

function get_dynamic_constraint(model, S, k, type)
    """
    build the constraint (4) restricted to a set of nodes
    
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
    constraint
        a new costraint to be added to the model
    """
    bound = ceil(length(S)/k)                           # round the division to the upper integer
    x1, x2, n = get_x1_x2_n(model)
    if type == 1
        con = @build_constraint(sum(x1[j,v] for v in 1:n, j in S if v ∉ S) >= bound)
    else
        con = @build_constraint(sum(x2[j,v] for v in 1:n, j in S if v ∉ S) >= bound)
    end
    return con
end

function build_constraint_23(model, v, w, S1, S2)
    """
    build the constraint (4) restricted to a set of nodes
    
    Parameters
    ---------
    model: Model
        MILP model of the problem
    v: int
        first index
    w: int
        second index
    S1: Array{Int64, 1}
        set of points S' obtainted by the mincut
    S2: Array{Int64, 1}
        set of points S obtainted by the mincut
    Return
    ---------
    constraint
        a new costraint to be added to the model
    """
    x1, x2, n = get_x1_x2_n(model)
    y1, y2 = get_y1_y2(model)
    con = @build_constraint(y1[v,w] - y2[v,w] - sum(x2[a,b] for a in S1, b in S2) <= 0)
    return con
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
    optimal_bound = objective_bound(model)
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

function get_y1_y2(model)
	"get y1 and y2 
	
	Parameters
	----------
	model: Model
		MILP model of the probel
	Return
	----------
	Float64
		pck matrix (y1)
	Float64
		dlv matrix (y2)
	"
	y1 = model[:y1]
    y2 = model[:y2]
	return y1, y2
end

function add_y_constraints(model)
    """
    Add the constraint from 6 to 14 to the model

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

    @variables(model, begin                         # precedence variables
        y1[2:n, 2:n], (Bin)                      
        y2[2:n, 2:n], (Bin)
    end)

    for u in 2:n    
        for v in 2:n
            for w in 2:n
                if u != v && u != w && v != w
                    @constraint(model, y1[u,v] + y1[v,u] + y1[v,w] + y1[w,v] + y1[u,w] + y1[w,u] >= 0)
                    @constraint(model, y2[u,v] + y2[v,u] + y2[v,w] + y2[w,v] + y2[u,w] + y2[w,u] >= 0)

                    @constraint(model, y1[u,v] + y1[v,w] - y1[u,w] <= 1)
                    @constraint(model, y2[u,v] + y2[v,w] - y2[u,w] <= 1)

                    @constraint(model, y1[u,v] + x1[v,w] - y1[u,w] <= 1)
                    @constraint(model, y2[u,v] + x2[v,w] - y2[u,w] <= 1)
                    
                    @constraint(model, y1[u,v] + x1[u,w] - y1[w,v] <= 1)
                    @constraint(model, y2[u,v] + x2[u,w] - y2[w,v] <= 1)
                end
            end
        end
    end

    for v in 2:n
        for w in 2:n
            if v != w
                @constraint(model, y1[v,w] + y1[w,v] <= 1)
                @constraint(model, y2[v,w] + y2[w,v] <= 1)

                @constraint(model, y1[v,w] + x1[v,w] <= 1)
                @constraint(model, y2[v,w] + x2[v,w] <= 1)

                @constraint(model, y1[v,w] + x1[w,v] <= 1)
                @constraint(model, y2[v,w] + x2[w,v] <= 1)

                @constraint(model, x1[1,v] + x1[1,w] - y1[v,w] - y1[w,v] <= 1)
                @constraint(model, x2[1,v] + x2[1,w] - y2[v,w] - y2[w,v] <= 1)
            end
        end
    end

    return model
end

function add_no_overlap_constraint(model)
    """
    Add the constraint 24 to obtain the no overlap variant

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
    y1, y2 = get_y1_y2(model)

    for u in 2:n
        for v in 2:n
            if u != v
                @constraint(model, y2[u,v] >= y1[u,v])
            end
        end
    end

    return model
end

function add_no_permutation_overlap_constraint(model)
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
    for v in 2:n
        for w in 2:n
            if v != w
                @constraint(model, 2x1[v,w] <= 2x2[v,w] + x2[v,1] + x2[1,w])
                @constraint(model, 2x2[v,w] <= 2x1[v,w] + x1[v,1] + x1[1,w])
                @constraint(model, 2x1[v,1] + 2x1[1,w] <= 2 + 2x2[v,w] + x2[v,1] + x2[1,w])
                @constraint(model, 2x2[v,1] + 2x2[1,w] <= 2 + 2x1[v,w] + x1[v,1] + x1[1,w])
            end
        end
    end
	return model
end

function add_no_permutation_no_overlap_constraint(model)
    """
    Add the constraints 16-17, 6-13, 24

    Parameters
    ----------
    model:
        base MILP model of the probel
    Return
    ----------
    Model
        the model with the added constraints
    """
    model = add_no_permutation_overlap_constraint(model)
    model = add_y_constraints(model)
    model = add_no_overlap_constraint(model)
    return model
end

function add_permutation_overlap_constraint(model)
    """
    Add the constraints 6-14 and 18

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

    model = add_y_constraints(model)

    y1, y2 = get_y1_y2(model)

    for v in 2:n
        for w in 2:n
            if v != w
                @constraint(model, y1[v,w] + y2[w,v] <= 1)
            end
        end
    end
    return model
end

function add_permutation_no_overlap_constraint(model)
    """
    Add the constraints 6-14, 18 and 24

    Parameters
    ----------
    model:
        base MILP model of the probel
    Return
    ----------
    Model
        the model with the added constraints
    """
    model = add_permutation_overlap_constraint(model)
    model = add_no_overlap_constraint(model)
    return model
end

function add_pickup_permutation_overlap_constraint(model)
    """
    Add the constraints 6-14, 22-23

    Parameters
    ----------
    model:
        base MILP model of the probel
    Return
    ----------
    Model
        the model with the added constraints
    """
    model = add_permutation_overlap_constraint(model)
    MOI.set(model, MOI.LazyConstraintCallback(), callback_c4_c23)
    return model
end

function add_pickup_permutation_no_overlap_constraint(model)
    """
    Add the constraints 6-14, 22-24

    Parameters
    ----------
    model:
        base MILP model of the probel
    Return
    ----------
    Model
        the model with the added constraints
    """
    model = add_pickup_permutation_overlap_constraint(model)
    model = add_no_overlap_constraint(model)
    return model
end

function add_delivery_permutation_overlap_constraint(model)
    """
    Add the constraints

    Parameters
    ----------
    model:
        base MILP model of the probel
    Return
    ----------
    Model
        the model with the added constraints
    """
    println("Not yet implemented!")
    exit(-1)
end

function add_delivery_permutation_no_overlap_constraint(model)
    """
    Add the constraints 

    Parameters
    ----------
    model:
        base MILP model of the probel
    Return
    ----------
    Model
        the model with the added constraints
    """
    println("Not yet implemented!")
    exit(-1)
end
