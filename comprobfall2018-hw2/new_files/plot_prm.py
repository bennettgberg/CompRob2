import matplotlib.pyplot as plt
import numpy as np

#COMPUTATION TIME VS. PATH QUALITY PLOT
for n in range(50, 151, 25):
    for prm in ["PRMstar_N=","PRM_N="]:
        filename = prm + str(n) + "_data.txt"
        infile = open(filename, "r")
        comptime=[]
        pathqual=[]        
        for line in infile:
            words = line.split()
            if len(words) < 2:
                break
            comptime.append(float(words[0]))
            pathqual.append(float(words[1]))
        avgComp=np.mean(comptime)
        avgQual=np.mean(pathqual)
        if prm == "PRM_N=": 
            plt.plot(avgComp, avgQual, 'b.',markersize=10)
            #plt.errorbar(avgComp, avgQual,xerr=0, yerr=np.std(pathqual),fmt='o',ecolor='b')
        else:# prm == "PRMstar_N=": 
            plt.plot(avgComp, avgQual, 'r.',markersize=10)
            #plt.errorbar(avgComp, avgQual,xerr=0, yerr=np.std(pathqual),fmt='o',ecolor='r')
        infile.close()
plt.title("Computation time vs path distance for PRM (set k, blue) vs PRM* (k=log2(N), red)")
plt.ylabel("Path Quality (-distance)")
plt.xlabel("Computational Time (# collision checks)")
plt.show()

#PATH QUALITY VS SIZE OF ROADMAP PLOT
for n in range(50, 151, 25):
    for prm in ["PRMstar_N=","PRM_N="]:
        filename = prm + str(n) + "_data.txt"
        infile = open(filename, "r")
        comptime=[]
        pathqual=[]
        for line in infile:
            words = line.split()
            if len(words) < 2: 
                break
            comptime.append(float(words[0]))
            pathqual.append(float(words[1]))
        avgComp=np.mean(comptime)
        avgQual=np.mean(pathqual)
        if prm == "PRM_N=": 
            plt.plot(n, -avgQual, 'b.', label="PRM",markersize=10)
            #plt.errorbar(n, avgQual,xerr=0, yerr=np.std(pathqual),fmt='bo',color='b')
        else: 
            plt.plot(n, -avgQual, 'r.', label="PRM*",markersize=10)
            #plt.errorbar(n, avgQual,xerr=0, yerr=np.std(pathqual),fmt='ro',color='r')
        infile.close()
plt.title("Path Distance vs size of roadmap for PRM (set k, blue) vs PRM* (k=log2(N), red)")
plt.ylabel("Path Quality (-distance)")
plt.xlabel("Number of Nodes in PRM")
plt.show()

#COMPUTATION TIME VS SIZE OF ROADMAP PLOT
for n in range(50, 151, 25):
    for prm in ["PRMstar_N=","PRM_N="]:
        filename = prm + str(n) + "_data.txt"
        infile = open(filename, "r")
        comptime=[]
        pathqual=[]        
        for line in infile:
            words = line.split()
            if len(words) < 2: 
                break
            comptime.append(float(words[0]))
            pathqual.append(float(words[1]))
        avgComp=np.mean(comptime)
        avgQual=np.mean(pathqual)            
        if (prm == "PRM_N="):
            plt.plot(n,avgComp, 'b.', label="PRM",markersize=10)
            #plt.errorbar(n, avgComp, yerr=np.std(comptime),fmt='o',ecolor='b')
        else: 
            plt.plot(n,avgComp, 'r.', label="PRM*",markersize=10)
            #plt.errorbar(n, avgComp, yerr=np.std(comptime),fmt='o',ecolor='r')
        infile.close()
plt.title("Computation time vs size of roadmap for PRM (set k,blue) vs PRM* (k=log2(N),red)")
plt.ylabel("Computational Time (number of collision checks)")
plt.xlabel("Number of Nodes in PRM")
plt.show()
