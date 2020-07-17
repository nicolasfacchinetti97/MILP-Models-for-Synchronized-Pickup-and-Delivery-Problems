include("tsp.jl")
include("opt.jl")

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


ptour, dtour, x1, x2 = try
    solve(model)
catch e
    println(e.msg)
end

println("Exiting")