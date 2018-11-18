import matplotlib.pyplot as plt

#COMPUTATION TIME VS. PATH QUALITY PLOT
for n in range(25, 151, 25):
    for prm in ["PRM_N", "PRMstar_N"]:
        filename = prm + str(n) + "_data.txt"
        infile = open(filename, "r")
        for line in infile:
            words = line.split()
            comptime = float(words[0])
            pathqual = float(words[1])
            if prm == "PRM_N": plt.plot(comptime, pathqual, 'b.', label="PRM")
            else: plt.plot(comptime, pathqual, 'r.', label="PRM*")
        infile.close()
plt.legend()
plt.title("Computation time vs path quality for PRM (constant k) vs PRM* (k=log2(N))")
plt.show()

#PATH QUALITY VS SIZE OF ROADMAP PLOT
for n in range(25, 151, 25):
    for prm in ["PRM_N", "PRMstar_N"]:
        filename = prm + str(n) + "_data.txt"
        infile = open(filename, "r")
        for line in infile:
            words = line.split()
            comptime = float(words[0])
            pathqual = float(words[1])
            if prm == "PRM_N": plt.plot(n, pathqual, 'b.', label="PRM")
            else: plt.plot(comptime, pathqual, 'r.', label="PRM*")
        infile.close()
plt.legend()
plt.title("Path quality vs size of roadmap for PRM (constant k) vs PRM* (k=log2(N))")
plt.show()

#COMPUTATION TIME VS SIZE OF ROADMAP PLOT
for n in range(25, 151, 25):
    for prm in ["PRM_N", "PRMstar_N"]:
        filename = prm + str(n) + "_data.txt"
        infile = open(filename, "r")
        for line in infile:
            words = line.split()
            comptime = float(words[0])
            pathqual = float(words[1])
            if prm == "PRM_N": plt.plot(n, pathqual, 'b.', label="PRM")
            else: plt.plot(comptime, pathqual, 'r.', label="PRM*")
        infile.close()
plt.legend()
plt.title("Computation time vs size of roadmap for PRM (constant k) vs PRM* (k=log2(N))")
plt.show()
