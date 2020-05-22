include("tsp.jl")

file_dir = "./istanze/"
filename = "33p00d.tsp"

p = [   [1 1],
        [2 3],
        [1 4],
        [4 3],
        [5 4]]
m = get_distance_matrix(p)
println(m)
