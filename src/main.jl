include("tsp.jl")
include("opt.jl")
include("fix_anomalies.jl")
include("io.jl")

# parameters
config.load_conf("conf.toml")

# parse the files to obtain the coords
println("Starting...\nParse points files.")
file_dir = config.get_file_dir()
pck_file = config.get_pck_file()
dlv_file = config.get_dlv_file()
read_n_node = config.get_read_n_nodes()
pck_points, dlv_points = parse_files(file_dir, pck_file, dlv_file, read_n_node)

# get distance matrix from the points
println("Compute distance matrix from points coords.")
to_round = config.get_to_round()
pck_matrix, dlv_matrix = get_distance_matrices(pck_points, dlv_points, to_round)

# check if the max number of nodes is > than the number of node of the instance
if size(pck_matrix, 1) <= read_n_node
    read_n_node = size(pck_matrix, 1)
end

print_log = config.get_print_log()
model_dump = config.get_model_dump()
# get the base MILP model of the problem (only constraints 1-3)
println("Get the base model of the problem.")
model = build_model(pck_matrix, dlv_matrix, print_log, model_dump)

overlap = config.get_overlap()
println("Setup the model for overlapping sequence: $overlap.")
if overlap
    model = add_no_permutation_overlap_constraint(model)
else
    model = add_no_permutation_no_overlap_constraint(model)
end

pi_tour, di_tour, x1, x2 = try
    solve(model, false)
catch e
    println(e.msg)
end  
println("Initial cost pickup $pi_tour, initial cost delivery $di_tour")

pck_k = config.get_pck_k()
dlv_k = config.get_dlv_k()
max_seconds = config.get_max_seconds()
# check and iteratively add the violated constraints 4 untill the are no more anomalies
println("-"^30, " Checking violated constraints ", "-"^30)
model, time = add_violated_constraints(model, x1, x2, pck_k, dlv_k, max_seconds)

# save the result of the instance
out_name = config.get_out_name()
save_instance(out_name, pck_file, model, read_n_node, pck_k, dlv_k, time)

pf_tour, df_tour, x1f, x2f = get_values(model)
println("Final cost pickup $pf_tour, final cost delivery $df_tour")
println("Initial cost pickup $pi_tour, initial cost delivery $di_tour")

if model_dump
    JuMP.write_to_file(model, "final_dump.lp")
    println("Model dump saved to file.\n")
end

# check solution integrity
if check_solution_integrity(x1f, x2f, pck_k, dlv_k)
    println("Integrity check passed\n")
else
    println("Integrity check not passed\n")
end

# export solution to dot file
save_dot = config.get_save_dot()
if save_dot
    create_dot_file(x1f, "pck.dot")
    create_dot_file(x2f, "dlv.dot")
    println("Saved .dot files.\n")
end

result_name = config.get_result_name()
println("Saving the results to file: $result_name")
save_solution(model, result_name)

println("Exiting")