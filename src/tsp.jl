function parse_file(file)
    """
    Parse a .tsp file with syntax according to TSPLib and return the list of points

    Parameters
    ---------
    file: string
        the string of the file to open
    Return
    ---------
    list
        a list of tuple cotaining the 2D coords of the points
    """
    points = Array{Float64, 2}[]                                    # array for the points
    start_save = false

    open(file, "r") do file
        for ln in eachline(file)
            if start_save == true
                ln = split(ln)[2:end]                               # extract coords from string
                coords = map(x->parse(Float64,x), ln)               # cast from string to float
                points = push!(points, [coords[1] coords[2]])
            end
            if ln == "NODE_COORD_SECTION"                           # find where start lists of coords
                start_save = true
            end
        end
    end

    return points
end

function parse_files(file_dir, pck_file, dlv_file)
    """
    Parse two .tsp file at a given directory

    Parameters
    ---------
    file_dir: string
        the directory cotaining the two parse_files
    pck_file: string
        filename of the pickup problem
    dlv_file: string
        filename of the delivery problem
    Return
    ---------
    tuple
        a tuple cotaining the two list of coords
    """
    file_location = string(file_dir, pck_file)
    pck_points = parse_file(file_location)
    file_location = string(file_dir, dlv_file)
    dlv_points = parse_file(file_location)
    return pck_points, dlv_points
end

function euclidean_distance(x, y)
    """
    Compute the euclidean distance between two points

    Parameters
    ---------
    x: tuple
        a tuple the 2D coord of the first point
    y: tuple
        a tuple the 2D coord of the second point
    Return
    ---------
    float
        the distance between the two points
    """
    return sqrt((x[1] - y[1])^2 + (x[2] - y[2])^2)
end

function correct_distances(matrix)
    """
    Correct the distances in a matrix using the Floyd-Warshall algorithm
    
    Parameters
    --------
    matrix: matrix
        matrix of distances between points to be corrected
    Return
    --------
    Matrix
        the matrix with the distances corrected
    """
    n = size(matrix)[1]
    for k in 1:n
        for i in 1:n
            for j in 1:n
                if matrix[i,j] > matrix[i,k] + matrix[k,j]
                    matrix[i,j] = matrix[i,k] + matrix[k,j]
                end
            end 
        end
    end
    return matrix
end

function get_distance_matrix(points, to_round)
    """
    Get the distance matrix between points

    Parameters
    ---------
    points: list
        a list of points in form of tuple
    to_round: boolean
        true if want the distances to be rounded to the closest interger, false otherwise
    Return
    ---------
    Matrix
        a matrix of float64 elements cotaining the distances between the given points
    """
    n = length(points)
    matrix = Matrix{Float64}(undef, n, n)                           # get a matrix of dimension n*n of Float
    for (i, x1) in enumerate(points)
        for (j, x2) in enumerate(points)
            dist = euclidean_distance(x1, x2)                       # euclidean distance between x1 and x2
            if to_round
                dist = round(dist)
            end
            matrix[i, j] = dist                                     # populate the row
        end
    end
    if to_round
        matrix = correct_distances(matrix)
    end
    return matrix
end

function get_distance_matrices(pck_points, dlv_points, to_round)
    """
    Get the distance matrices of the pickup and delivery

    Parameters
    ---------
    pck_points: list
        list of points of the pickup problem
    dlv_points: list
        list of points of the delivery problem
    to_round: boolean
        true if want the distances to be rounded to the closest interger, false otherwise
    Return
    ---------
    tuple
        a tuple of matrices
    """
    pck_matrix = get_distance_matrix(pck_points, to_round)
    dlv_matrix = get_distance_matrix(dlv_points, to_round)
    return pck_matrix, dlv_matrix
end

function check_solution_integrity(pck_m, dlv_m, pck_k, dlv_k)
    """
    Checking the integrity of pickup and delivery

    Parameters
    ---------
    pck_m: Array{Int64, 2}
        matrices of edges that describe the pickup solution
    pck_k: int
        maximum capacity of the pickup veichle
    dlv_m: Array{Int64, 2}
        matrices of edges that describe the delivery solution
    dlv_k: int
        maximum capacity of the delivery veichle 
    Return
    ---------
    boolean
        a boolean stating that if the solutions are valid or not
    """
    println("Checking pickup solution integrity...")
    pck = check_integrity(pck_m, pck_k)
    println("Checking delivery solution integrity...")
    dlv = check_integrity(dlv_m, dlv_k)
    return (pck && dlv)
end

function check_integrity(matrix, capacity)
    """
    Checking the integrity of a solution, all the nodes in a tour and capacity not exceed

    Parameters
    ---------
    matrix: Array{Int64, 2}
        matrices of edges that describe the solution
    capacity: int
        maximum capacity of the veichle
    Return
    ---------
    boolean
        a boolean stating that if the solution is valid or not
    """
    tours, excluded = find_connected_excluded_elements(matrix)

    if length(excluded) > 0
        println("Left out from the tours vertices: ", excluded)
        return false
    end
    for t in tours
        if length(t) > capacity
            println("Tour ", t, " exceed the capacity.")
            return false
        end
    end
    println("The solution is valid!")
    return true
end