include("tsp.jl")
include("opt.jl")
include("fix_anomalies.jl")

# parameters
pck_k = 3                               # pickup veichle dimension
dlv_k = 2                               # delivery veichle dimension
file_dir = "./istanze/"                 # folder containing istances
pck_file = "prova_p.tsp"                # pickup file
dlv_file = "prova_d.tsp"                # delivery file
to_round = true                         # round the value when find the euclidean dist
print_log = false                       # suppress logging of CPLEX
model_dump = true                       # for dumping to .lp file the model

# parse the files to obtain the coords
println("Starting...\nParse points files.")
pck_points, dlv_points = parse_files(file_dir, pck_file, dlv_file)

# get distance matrix from the points
println("Compute distance matrix from points coords.")
pck_matrix, dlv_matrix = get_distance_matrices(pck_points, dlv_points, to_round)

# get the base MILP model of the problem (only constraints 1-3)
println("Get the base model of the problem.")
model = build_model(pck_matrix, dlv_matrix, print_log, model_dump)
pi_tour, di_tour, x1, x2 = try
    solve(model)
catch e
    println(e.msg)
end  
println("Initial cost pickup ", pi_tour, ", initial cost delivery ", di_tour, "\n", x1, "\n", x2)

# check and iteratively add the violated constraints 4 untill the are no more anomalies
println("-"^30, " Checking violated constraints ", "-"^30)
model = add_violated_constraints(model, x1, x2, pck_k, dlv_k)

pf_tour, df_tour, x1f, x2f = get_values(model)
println("Final cost pickup ", pf_tour, ", final cost delivery ", df_tour)
println("Initial cost pickup ", pi_tour, ", initial cost delivery ", di_tour)

if model_dump
    JuMP.write_to_file(model, "final_dump.lp")
end

println("Exiting")