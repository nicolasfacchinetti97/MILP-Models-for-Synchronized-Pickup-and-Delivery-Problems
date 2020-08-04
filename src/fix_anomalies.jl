import Dates

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

function get_violated_constraint(model, anomaly, capacity, problem_type)
    """
    from an anomaly in the pickup or delivery problem get the corresponding violated constraint

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
    constraint:
        the previous violated constraint
    """
    type = anomaly[1]
    nodes = anomaly[2]
    
    println("Find a type $type anomaly, set of nodes that violate the constraint 4: $nodes")
    
    # get the constraint
    con = get_dynamic_constraint(model, nodes, capacity, problem_type)
    
    return con
end

function print_arr(arr)
    show(stdout, "text/plain", arr)
    println()
end

function check_constraints(model, m, k, type)
    """
    check the model if violate on of the constraint (4); if yes return the constraint, 0 otherwise


    Parameters
    ---------
    model: Model
        MILP model of the problem
    m: Array{Int64, 2}
        matrix of the arches describing the graph
    k: int
        capacity of the veichle
    type: int
        1 for pickup, 2 for delivery
    Return
    ---------
    con:
        a violated constraint of type 4
    int:
        0 if no violated constraint finded
    """
    res = detect_anomalies_in_tour(m, k)
    if res == 0
        println("No anomalies in tour")
        return 0
    else
        con = get_violated_constraint(model, res, k, type)
        return con
    end
end

function fix_model_with_heuristic(model, k_pck, k_dlv)
    x1, x2, n = get_x1_x2_n(model)

    println("Fixing pickup tour...")
    x1 = fix_tours_heuristic(x1, k_pck)

    println("Fixing delivery tour...")
    x2 = fix_tours_heuristic(x2, k_dlv)
    
    # asseganre x1 e x2 a modello

    return model    
end

function fix_tours_heuristic(matrix, k)
    tours, excluded = find_connected_excluded_elements(matrix)
    print_arr(matrix)
    while length(excluded) > 0
        matrix = fix_excluded_elements(matrix, excluded)
        tours, excluded = find_connected_excluded_elements(matrix)
    end

    print_arr(matrix)
    tours_e = [tour for tour in tours if length(tour) > k]
    while length(tours_e) > 0
        matrix = fix_tours_exceed_capacity(matrix, tours, k)
        tours, excluded = find_connected_excluded_elements(matrix)
        tours_e = [tour for tour in tours if length(tour) > k]
    end
    print_arr(matrix)

end

function fix_tours_exceed_capacity(matrix, tours, k)
    for tour in tours
        if length(tour) > k
            matrix = add_source_to_tour(matrix, tour[1], tour[k])
        end
    end

    return matrix
end

function fix_excluded_elements(matrix, elements)
    node = elements[1]
    tour = find_tour(node, node, matrix)
    first = tour[1]
    last = tour[end]

    matrix = add_source_to_tour(matrix, first, last)

    println("Added the tour starting from source with nodes: $tour")
    return matrix
end

function add_source_to_tour(matrix, first, last)
    matrix[1, first] = 1
    matrix[last, 1] = 1
    for i in 2:size(matrix, 1)
        matrix[last, i] = 0
    end

    return matrix
end