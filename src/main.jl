include("tsp.jl")
include("opt.jl")


file_dir = "./istanze/"
pck_file = "33p00p.tsp"
dlv_file = "33p00d.tsp"

# parse the file for obtain the coords
pck_points = parse_file(file_dir, pck_file)
dlv_points = parse_file(file_dir, dlv_file)
# get distance matrix from the points
pck_matrix = get_distance_matrix(pck_points)
dlv_matrix = get_distance_matrix(dlv_points)
# get capacity of the two veichle
pck_k = 3
dlv_k = 2
