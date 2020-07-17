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
file_location = string(file_dir, pck_file)
pck_points = parse_file(file_location)

file_location = string(file_dir, dlv_file)
dlv_points = parse_file(file_location)

println("Compute distance matrix from points coords.")

# get distance matrix from the points
pck_matrix = get_distance_matrix(pck_points, to_round)
dlv_matrix = get_distance_matrix(dlv_points, to_round)

println("Get the base model of the problem.")
model = build_model(pck_matrix, dlv_matrix)


ptour, dtour, x1, x2 = try
    solve(model)
catch e
    println(e.msg)
end

println(ptour)
println(dtour)

println("Exiting")
