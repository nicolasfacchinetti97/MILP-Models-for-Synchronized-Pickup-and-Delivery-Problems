import Distances

function parse_file(file_dir, filename)
    cd(file_dir)                                                    # change the directory

    points = Array{Float64, 2}[]                                    # array for the points
    start_save = false

    open(filename, "r") do file
        for ln in eachline(file)
            global start_save
            global points
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

function get_distance_matrix(points)
    matrix = Array{Array{Float64, 1}, 1}()
    for x1 in points
        row = Array{Float64, 1}()                                   # an array for the row
        for x2 in points
            elem = sqrt((x1[1] - x2[1])^2 + (x1[2] - x2[2])^2)      # euclidean distance between x1 and x2
            push!(row, elem)                                        # populate the row
        end
        push!(matrix, row)                                          # push the row to the distance matrix
    end
    #for r in matrix                                                # debug
    #    println(r)
    #end
    return matrix
end
