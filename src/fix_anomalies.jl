function find_tour(start_node, end_node, matrix)
    """
    find a tour in the graph using a starting node and a end node in a matrix of edges

    Parameters
    ---------
    start_node: int
        the index of the starting node of the plan
    end_node: int
        the index of the ending node of the plan
    matrix: Array{Int64, 2}
        matrix of edges in the graph
    Return
    ---------
    array
        collection of visited nodes
    """
    n = size(matrix, 1)
    tour = [start_node]
    next = [i for i in 1:n if matrix[start_node, i] > 0.5][1]
    while next != end_node
        push!(tour, next)
        next = [i for i in 1:n if matrix[next, i] > 0.5][1]
    end
    return tour
end

function detect_anomalies_in_tour(matrix, capacity)
    """
    Return the first anomaly of type 1 or 2 in the graph described by the given matrix of edges
    type 1 anomaly: tours no connected to the source
    type 2 anomaly: connected tours that exceed the capacity of the veichle

    Parameters
    ---------
    matrix: Array{Int64, 2}
        matrix of the archs of the graph
    capacity: Int64
        maximum capacity of the veichle
    Return
    ---------
    tuple    
        1 first type anomaly, 2 second type anomaly     
        list of first nodes affected by one of the two anomaly
    0
        if no anomaly detected
    """
    tours, excluded = find_connected_excluded_elements(matrix)


    # type 1 anomaly - disconnected tour
    for n in excluded
        return (1, find_tour(n, n, matrix))
    end

    # type 2 anomaly - connected tour that exceed the capacity
    for t in tours
        if length(t) > capacity
            return (2, t)
        end
    end
    return 0
end


function find_connected_excluded_elements(matrix)
    """
    find connected tours and no connected elements in a graph

    Parameters
    ---------
    matrix: Array{Int64, 2}
        matrix of arches that describe the graph
    Return
    ---------
    tuple:
        tours: Array{Array{Int64, 1}, 1}
            array of tours that start from source
        excluded: Array{Int64, 1}
            array of excluded elements from tours that start from the source
    """
    n = size(matrix, 1)
    start_nodes = [i for i in 1:n if matrix[1,i] > 0.5]              # find nodes reachable from source

    tours = Array{Array{Int64,1},1}()                               # collect all the tours reachable from source
    for n in start_nodes
        tour = find_tour(n, 1, matrix)
        push!(tours, tour)
    end
    included = Array{Int64, 1}()
    for tour in tours                                               # find all the nodes in tour
        union!(included, tour)
    end
    excluded = setdiff(2:n, included)                               # nodes not in tour starting at depot
    return tours, excluded
end

function add_violated_constraint(model, anomaly, capacity, problem_type)
    """
    get an anomaly in the pickup or delivery problem and add the corresponding constraint to the model

    Parameters
    ---------
    model: Model
        MILP model of the problem
    anomaly: tuple
        int
            type of anomaly (1 or 2)
        Array{Int64, 1}
            set of nodes involved in the anomaly
    capacity: int
        capacity of the veichle
    problem_type: int
        1 for pickup, 2 for delivery
    Return
    ---------
    Model:
        model of the problem with the added constraint
    """
    type = anomaly[1]
    nodes = anomaly[2]
    
    println("Type $type anomaly, set of nodes that violate the constraint 4: $nodes")
     
    model = add_dynamic_constraint(model, nodes, capacity, problem_type)
    p_tour, d_tour, x1, x2 = try
        solve(model)
    catch e
        println(e.msg)
    end
    println("New cost pickup $p_tour, new cost delivery $d_tour")

    return model
end

function print_arr(arr)
    show(stdout, "text/plain", arr)
    println()
end

function add_violated_constraints(model, m_pck, m_dlv, k_pck, k_dlv)
    """
    search repeatedly for anomalies in the pickup and delivery tour and add them to the model

    Parameters
    ---------
    model: Model
        MILP model of the problem
    m_pck: Array{Int64, 2}
        matrix of the arches for the pickup
    m_dlv: Array{Int64, 2}
        matrix of the arches for the delivery
    k_pck: int
        capacity of the pickup veichle
    k_dlv: int
        capacity of the delivery veichle
    Return
    ---------
    Model:
        final MILP model with all the constraint of type 4
    """
    while true
        done_pck = false
        done_dlv = false

        res_pck = detect_anomalies_in_tour(m_pck, k_pck)
        if res_pck == 0
            println("No more anomalies in pickup")
            done_pck = true
        else
            println("Find an anomaly in pickup tour!")
            model = add_violated_constraint(model, res_pck, k_pck, 1)
        end

        res_dlv = detect_anomalies_in_tour(m_dlv, k_dlv)
        if res_dlv == 0
            println("No more anomalies in delivery")
            done_dlv = true
        else
            println("Find an anomaly in delivery tour!")
            model = add_violated_constraint(model, res_dlv, k_dlv, 2)
        end
        println("\n")

        if done_pck && done_dlv
            return model
        else
            m_pck = get_x1(model)
            m_dlv = get_x2(model)
        end
    end
end
