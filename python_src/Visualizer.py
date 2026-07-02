import numpy as np
import matplotlib.pyplot as plt

CSV_PATH = r"..\simulation_results.csv"

file = np.genfromtxt(CSV_PATH, delimiter=',')
file = file[1:,:]
print(file)
tout = file[:,0]
pout = file[:,1]
vout = file[:,2]

plt.figure()
plt.plot(tout, pout)
plt.xlabel("Time")
plt.ylabel("Position")

#plt.plot(tout, vout)
# plt.xlabel("Time")
# plt.ylabel("Velocity/position")
# plt.legend(["Position", "Velocity"])
# plt.title("Spring Simulation")
plt.grid()
plt.show()
