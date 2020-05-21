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

function get_distance_matrix(point)

end


