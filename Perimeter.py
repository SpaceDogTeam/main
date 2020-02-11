from math import cos, sin, pi, sqrt
import matplotlib.pyplot as plt

L = 44

def convertAngle(angle):
    conv = (angle * 2*pi) / 360
    return conv

def DirectPos(alpha, beta):
    alpha = convertAngle(alpha)
    beta = convertAngle(beta)

    rbeta = 2*pi - beta
    delta = rbeta - alpha
    X = L*(cos(alpha) - cos(delta))
    Y = L*(sin(alpha) + sin(delta))
    return(X, Y)

def untuple(tlist):
    xlist = []
    ylist = []
    for (elem, elem2) in tlist:
        xlist += [elem]
        ylist += [elem2]
    return (xlist, ylist)

def highlight(tlist):
    X, Y = untuple(tlist)
    plt.plot(X, Y, "ro")

def drawLeg(alpha, beta, color):
    alpha_rad = convertAngle(alpha)
    beta_rad = convertAngle(beta)

    X = [0]
    Y = [0]
    X += [44*cos(alpha_rad)]
    Y += [44*sin(alpha_rad)]
    x0, y0 = DirectPos(alpha, beta)
    X += [x0]
    Y += [y0]

    distance_x = X[2] - X[1]
    distance_y = Y[2] - Y[1]
    distance = sqrt(distance_x**2 + distance_y**2)
    print(distance)

    plt.plot(X, Y, color)

dots = []
for a in range(77, 95):
    positions = []
    for b in range(306, 338):
        positions += [DirectPos(a, b)]
    dots += [positions]

for elem in dots:
    X, Y = untuple(elem)
    plt.plot(X, Y, "go")

highlight(dots[0])
highlight(dots[-1])
drawLeg(77, 306, "k*-")
#drawLeg(77, 337, "b*-")
drawLeg(94, 306, "r*-")
#drawLeg(94, 337, "g*-")
plt.draw()
plt.show()

