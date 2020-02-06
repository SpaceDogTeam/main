from math import pi, sin
import matplotlib.pyplot as plt


def SineComp():
    samplerate = 10
    iterations = 1000

    Xcoords = []
    Ycoords = []
    for elem in range(iterations):
        Xcoords += [elem*samplerate*0.001]
        Ycoords += [9 * sin(2 * pi * (Xcoords[elem]+2.5) / 10) + 85]

    return(Xcoords, Ycoords)

def PIDTuning():
    iterations = 1000

    Xcoords = []
    Ycoords = []
    for elem in range(iterations):
        Xcoords += [elem*0.01]
        if(elem < 10):
            Ycoords += [94]
        else:
            Ycoords += [84]

    return(Xcoords, Ycoords)


Xcoords, Ycoords = SineComp()
#Xcoords, Ycoords = PIDTuning()
with open("Dataset7.txt", 'r') as file:
    Mcoords_l = file.read()
    file.close()

Mcoords = [float(elem) for elem in Mcoords_l.split('\n')]

plt.xlabel("X-axis")
plt.ylabel("Y-axis")
plt.plot(Xcoords, Ycoords, "go-")
plt.plot(Xcoords, Mcoords, "kd-") #measures
plt.grid()
plt.draw()
plt.show()