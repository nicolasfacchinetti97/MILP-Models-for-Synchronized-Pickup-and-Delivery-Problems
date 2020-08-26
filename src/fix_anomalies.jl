import Dates

# for the calculus of the mincut
using LightGraphsFlows
import LightGraphs
const lg = LightGraphs

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

function check_constraint_4(model, m, k, type)
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

function check_constraint_23(model, md, y1, y2)
    """
    check the model if violate one of the constraint (23); if yes return the constraint, 0 otherwise

    Parameters
    ---------
    model: Model
        MILP model of the problem
    md: Array{Int64, 2}
        matrix of the arches describing the delivery graph
    y1: Array{Int64, 2}
        matrix of precedence y1
    y2: Array{Int64, 2}
        matrix of precedence y2
    Return
    ---------
    con:
        a violated constraint of type 23
    int:
        0 if no violated constraint finded
    """
    n = size(md, 1)
    tol = get_optimizer_attribute(model, "CPX_PARAM_EPINT")
    for v in 2:n
        for w in 2:n
            if v != w
                nodes = setdiff(1:n, w)
                graph, weights = build_directed_graph(nodes, md, n)
                # calculate the mincut and return the two set of nodes
                set1, set2, value = LightGraphsFlows.mincut(graph, 1, v, weights, LightGraphsFlows.PushRelabelAlgorithm())
                
                if (y1[v,w] - y2[v,w]) > value + tol
                    set2 = setdiff!(set2, w)
                    println("Find a violated constraint 23, w: $w, S': $set1, S: $set2")

                    return build_constraint_23(model, v, w, set1, set2)
                end
            end
        end
    end
    println("No violated constraint 23 found!")
    return 0
end

function build_directed_graph(nodes, m, n)
    """
    buil a graph to compute the mincut

    Parameters
    ----------
    nodes: Array{Int64, 1}
        list of nodes to put in the graph
    m: Array{Int64, 2}
        matrix of arches describing the tour
    n: int
        number of nodes of the graph
    Return
    ----------
    tuple:
        graph: DiGraph
            directed graph with edges
        capacity_matrix: Array{Int, 2}
            array with edge values for the graph
    """
    graph = lg.DiGraph(n)                                   # my graph
    capacity_matrix = zeros(Int, n, n)                      # array with edges values                  
    for v in nodes
        for w in nodes
            lg.add_edge!(graph, v, w)
            capacity_matrix[v,w] = (m[v,w] >= 0.5 ? 1 : 0) # for the non integer values returned by cplex
        end
    end
    return graph, capacity_matrix
end