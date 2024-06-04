import matplotlib.pyplot as plt

# # Dados
# num_grafos = [5, 6, 7, 8, 9, 10, 11]
# best_path = [0, 0, 4, 37, 384, 4235, 62351]
# best_path_omp = [0, 0, 3, 29, 309, 3547, 44155]
# best_path_mpi = [10, 10, 11, 11, 103, 1295, 13443]
# heuristic_path = [0, 0, 0, 0, 0, 0, 1]
# heuristic_omp = [0, 0, 0, 0, 1, 80, 2]
# heuristic_mpi = [0, 0, 0, 0, 0, 10, 3]

# # Deletando primeiros 3 elementos de cada lista
# num_grafos = num_grafos[3:]
# best_path = best_path[3:]
# best_path_omp = best_path_omp[3:]
# best_path_mpi = best_path_mpi[3:]
# heuristic_path = heuristic_path[3:]
# heuristic_omp = heuristic_omp[3:]
# heuristic_mpi = heuristic_mpi[3:]

# # Gráfico sem escala logarítmica
# plt.figure(figsize=(14, 7))

# plt.subplot(1, 2, 1)
# plt.plot(num_grafos, best_path, label="Best Path")
# plt.plot(num_grafos, best_path_omp, label="Best Path OMP")
# plt.plot(num_grafos, best_path_mpi, label="Best Path MPI")
# plt.plot(num_grafos, heuristic_path, label="Heuristic Path")
# plt.plot(num_grafos, heuristic_omp, label="Heuristic OMP")
# plt.plot(num_grafos, heuristic_mpi, label="Heuristic MPI")

# plt.xlabel("Número de Grafos")
# plt.ylabel("Tempo (ms)")
# plt.title("Vehicle Routing Problem - Linear Scale")
# plt.legend()
# plt.grid(True)

# # Gráfico com escala logarítmica
# plt.subplot(1, 2, 2)
# plt.plot(num_grafos, best_path, label="Best Path")
# plt.plot(num_grafos, best_path_omp, label="Best Path OMP")
# plt.plot(num_grafos, best_path_mpi, label="Best Path MPI")
# plt.plot(num_grafos, heuristic_path, label="Heuristic Path")
# plt.plot(num_grafos, heuristic_omp, label="Heuristic OMP")
# plt.plot(num_grafos, heuristic_mpi, label="Heuristic MPI")

# plt.yscale("log")
# plt.xlabel("Número de Grafos")
# plt.ylabel("Tempo (ms) - Escala Log")
# plt.title("Vehicle Routing Problem - Logarithmic Scale")
# plt.legend()
# plt.grid(True)

# plt.tight_layout()
# plt.show()

num_grafos = [5, 6, 7, 8, 9, 10, 11]
global_cost = [526, 264, 604, 785, 943, 846, 896]
heuristic_cost = [526, 378, 640, 785, 993, 1012, 896]

# Gráfico sem escala logarítmica

plt.figure(figsize=(14, 7))

plt.subplot(1, 2, 1)
plt.plot(num_grafos, global_cost, label="Global Cost")
plt.plot(num_grafos, heuristic_cost, label="Heuristic Cost")

plt.xlabel("Número de Grafos")
plt.ylabel("Custo")
plt.title("Vehicle Routing Problem - Linear Scale")
plt.legend()
plt.grid(True)

plt.tight_layout()
plt.show()
