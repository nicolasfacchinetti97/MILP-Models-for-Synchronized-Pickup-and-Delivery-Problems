include("tsp.jl")
include("opt.jl")
include("fix_anomalies.jl")
include("io.jl")

# TODO fix all the call to config with the use of the module
println("Load configurations from file...")
config.load_conf("conf.toml")
file_dir = config.get_file_dir()
pck_file = config.get_pck_file()
dlv_file = config.get_dlv_file()
read_n_node = config.get_read_n_nodes()
to_round = config.get_to_round()
max_seconds = config.get_max_seconds()
print_log = config.get_print_log()
model_dump = config.get_model_dump()
overlap = config.get_overlap()
pck_k = config.get_pck_k()
dlv_k = config.get_dlv_k()
max_seconds = config.get_max_seconds()
out_name = config.get_out_name()
save_dot = config.get_save_dot()
result_name = config.get_result_name()
permutation = config.get_permutation()

# parse the files to obtain the coords
println("Parse points files.")
pck_points, dlv_points = parse_files(file_dir, pck_file, dlv_file, read_n_node)

# get distance matrix from the points
println("Compute distance matrix from points coords.")
pck_matrix, dlv_matrix = get_distance_matrices(pck_points, dlv_points, to_round)

# check if the max number of nodes is > than the number of node of the instance
if size(pck_matrix, 1) <= read_n_node
    read_n_node = size(pck_matrix, 1)
end

# get the base MILP model of the problem (only constraints 1-3)
println("Get the base model of the problem.")
model = build_model(pck_matrix, dlv_matrix, max_seconds, print_log)

print("Flavor of the problem: ")
if permutation == 1
    println("no permutation.")
elseif permutation == 2
    println("permutation.")
elseif permutation == 3
    println("pickup permutation.")
elseif permutation == 4
    println("delivery permutation.")
end
println("Setup the model for overlapping sequence? $overlap.")

if permutation == 1 && overlap
    model = add_no_permutation_overlap_constraint(model)
elseif permutation == 1 && !overlap
    model = add_no_permutation_no_overlap_constraint(model)
elseif permutation == 2 && overlap
    model = add_permutation_overlap_constraint(model)
elseif permutation == 2 && !overlap
    model = add_permutation_no_overlap_constraint(model)
elseif permutation == 3 && overlap
    model = add_pickup_permutation_overlap_constraint(model)
elseif permutation == 3 && !overlap
    model = add_pickup_permutation_no_overlap_constraint(model)
elseif permutation == 4 && overlap
    model = add_delivery_permutation_overlap_constraint(model)
elseif permutation == 4 && !overlap
    model = add_delivery_permutation_no_overlap_constraint(model)
end

# if model_dump
#     JuMP.write_to_file(model, "init_dump.lp")
# end

pi_tour, di_tour, x1, x2, time = try
    solve(model, false)
catch e
    println(e.msg)
    # solution not found
    println("No base solution found, skipping problem $pck_file + $dlv_file...")
    exit(-1)
end
toursp, _ = find_connected_excluded_elements(x1)
toursd, _ = find_connected_excluded_elements(x2) 
println("\nCost pickup: $pi_tour, cost delivery: $di_tour.\nPickup tour $toursp\nDelivery tour $toursd\n")

# save the result of the instance
save_instance(out_name, pck_file, permutation, overlap, model, read_n_node, pck_k, dlv_k, time)

# if model_dump
#     JuMP.write_to_file(model, "final_dump.lp")
#     println("Model dump saved to file.\n")
# end

# check solution integrity
if check_solution_integrity(x1, x2, pck_k, dlv_k, overlap)
    println("Integrity check passed\n")
else
    println("Integrity check not passed\n")
end

# export solution to dot file
if save_dot
    create_dot_file(x1, "pck.dot")
    create_dot_file(x2, "dlv.dot")
    println("Saved .dot files.\n")
end

println("Saving the results to file: $result_name")
save_solution(model, result_name)

println("Exiting")