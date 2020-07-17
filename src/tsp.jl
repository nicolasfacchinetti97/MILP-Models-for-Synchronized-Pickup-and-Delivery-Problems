
function parse_file(file)
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

function get_distance_matrix(points, to_round)
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
    return matrix
end

function euclidean_distance(x, y)
    return sqrt((x[1] - y[1])^2 + (x[2] - y[2])^2)
end