include("tsp.jl")
include("opt.jl")

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
    next = [i for i in 1:n if matrix[start_node, i] == 1][1]
    while next != end_node
        push!(tour, next)
        next = [i for i in 1:n if matrix[next, i] == 1][1]
    end
    return tour
end

function detect_anomaly_in_tour(matrix, capacity)
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
        list of first nodes affected by one of the two anomaly or 0 if no anomaly detected
    """
    tours, excluded = find_nodes(matrix)
    # type 1 anomaly - disconnected tour
    for n in excluded
        return find_tour(n, n, matrix)
    end

    # type 2 anomaly - connected tour that exceed the capacity
    for t in tours
        if length(t) > capacity
            return t
        end
    end
    return 0
end


function find_nodes(matrix)
    """
    find no connected elements and connected tour in a graph

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
    start_nodes = [i for i in 1:n if matrix[1,i] == 1]              # find nodes reachable from source
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

function add_violated_constraints(model, matrix, capacity)
    while true
        res = detect_anomaly_in_tour(matrix, capacity)
        if res == 0
            break
        end
        println("Find a set on nodes that violate the constraint 4: ", res)
        model = add_dynamic_constraint(model, res, capacity)
        p_tour, d_tour, x1, x2 = try
            solve(model)
        catch e
            println(e.msg)
        end
        println("New cost pickup: ", p_tour)  
    end
end

# parameters
pck_k = 3                               # pickup veichle dimension
dlv_k = 2                               # delivery veichle dimension
file_dir = "./istanze/"                 # folder containing istances
pck_file = "prova_p.tsp"                # pickup file
dlv_file = "prova_d.tsp"                # delivery file
to_round = true                         # round the value when find the eclidean dist

println("Starting...\nParse points files.")

# parse the files for obtain the coords
pck_points, dlv_points = parse_files(file_dir, pck_file, dlv_file)

println("Compute distance matrix from points coords.")

# get distance matrix from the points
pck_matrix, dlv_matrix = get_distance_matrices(pck_points, dlv_points, to_round)

println("Get the base model of the problem.")
model = build_model(pck_matrix, dlv_matrix)

pi_tour, di_tour, x1, x2 = try
    solve(model)
catch e
    println(e.msg)
end  

println(string("Initial cost pickup ", pi_tour, ", initial cost delivery ", di_tour))

add_violated_constraints(model, x1, pck_k)

println("Exiting")