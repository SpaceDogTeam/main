from math import pi, sin
import matplotlib.pyplot as plt

samplerate = 10
iterations = 1000

Xcoords = []
Ycoords = []
for elem in range(iterations):
    Xcoords += [elem*samplerate*0.001]
    Ycoords += [9 * sin(2 * pi * (Xcoords[elem]+2.5) / 10) + 85]

with open("Dataset1.txt", 'r') as file:
    Mcoords_l = file.read()
    file.close()

Mcoords = [float(elem) for elem in Mcoords_l.split('\n')]

plt.xlabel("X-axis")
plt.ylabel("Y-axis")
plt.plot(Xcoords, Ycoords, "go")
plt.plot(Xcoords, Mcoords, "kd-") #measures
plt.grid()
plt.draw()
plt.show()