module config
    using TOML

    conf = Dict()

    function load_conf(filename)
        """
        Load the parameters of the program from a file

        Parameters
        ---------
        filename: string
            the name of the config filem with his relative path
        """
        global conf = TOML.parsefile(filename)
    end

    function get_pck_k()
        return conf["pck_k"]
    end

    function get_dlv_k()
        return conf["dlv_k"]
    end

    function get_file_dir()
        return conf["file_dir"]
    end  
        
    function get_pck_file()
        return conf["pck_file"]
    end

    function get_dlv_file()
        return conf["dlv_file"]
    end

    function get_to_round()
        return conf["to_round"]
    end

    function get_file_dir()
        return conf["file_dir"]
    end

    function get_print_log()
        return conf["print_log"]
    end  
    
    function get_model_dump()
        return conf["model_dump"]
    end  

    function get_save_dot()
        return conf["save_dot"]
    end
    
    function get_result_name()
        return conf["result_name"]
    end

    function get_read_n_nodes()
        return conf["read_n_nodes"]
    end

    function get_max_seconds()
        return conf["max_seconds"]
    end

    function get_out_name()
        return conf["out_name"]
    end

    function get_overlap()
        return conf["overlap"]
    end
end

function save_instance(filename, name, model, n, k_p, k_d, time)
    """
    Write on a file a row in the format "instance_name n kp kd 3b-1 3b-2 time"

    Parameters
    ----------
    filename: string
        name of the output file
    name: string
        name of the instance
    model: Model
        solved MILP model of the probel
    n: int
        nuber of nodes
    k_p: int
        pickup capacity
    k_d: int
        delivery capacity
    time: int
        time in seconds elapsed
    """
    best_value, lower_bound = get_opt(model)
    open(filename, "a") do f
        write(f, "$name $n $k_p $k_d $best_value $lower_bound $time \n")
    end
end

function save_solution(model, filename)
    """
    Save the solution of the program to file

    Parameters
    ----------
    model: Model
        MILP model of the probel resolved
    filename: string
        name of the file used to print the results
    Return
    ----------
    boolean
        true if the function is executed
    """
    cost_pck, cost_dlv, matrix_pck, matrix_dlv = get_values(model)
    tour_pck, _ = find_connected_excluded_elements(matrix_pck)
    tour_dlv, _ = find_connected_excluded_elements(matrix_dlv)
    open(filename, "w") do f
        write(f, "PICKUP\n")
        write(f, "COST\n")
        write(f, "$cost_pck \n")
        write(f, "TOUR\n")
        writedlm(f, tour_pck)
        write(f, "MATRIX\n")
        writedlm(f, matrix_pck)
        write(f, "DELIVERY\n")
        write(f, "COST\n")
        write(f, "$cost_dlv \n")
        write(f, "TOUR\n")
        writedlm(f, tour_pck)
        write(f, "MATRIX\n")
        writedlm(f, matrix_pck)
    end
    
    return true
end

function create_dot_file(matrix, filename)
    """
    Create a .dot file that describe a graph

    Parameters
    ----------
    matrix: Array{Int64, 2}
        matrix of edges that describe a graph
    filename: string
        string of the file
    Return
    ----------
    """
    n,m = size(matrix)                          # get the size of the matrix
    open(filename, "w") do f
        write(f, "digraph graphname {\n")
        for i in 1:n
            for j in 1:m
                if matrix[i,j] == 1
                    write(f, "$i -> $j; \n")
                end
            end
        end
        write(f, "}")
    end
end

function parse_file(file, max_node)
    """
    Parse a .tsp file with syntax according to TSPLib and return the list of points

    Parameters
    ---------
    file: string
        the string of the file to open
    max_node: int
        max number of nodes to read from the files
    Return
    ---------
    list
        a list of tuple cotaining the 2D coords of the points
    """
    points = Array{Float64, 2}[]                                    # array for the points
    start_save = false
    i = 0
    open(file, "r") do file
        for ln in eachline(file)
            if start_save == true
                if i == max_node                                    # return if read max number of nodes
                    return points
                end
                ln = split(ln)[2:end]                               # extract coords from string
                coords = map(x->parse(Float64,x), ln)               # cast from string to float
                points = push!(points, [coords[1] coords[2]])
                i += 1
            end
            if ln == "NODE_COORD_SECTION"                           # find where start lists of coords
                start_save = true
            end
        end
    end

    return points
end
