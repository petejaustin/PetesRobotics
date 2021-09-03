import numpy as np 
from numpy import linalg as LA
import math

x = [0., 0., 0.333]
y = [0., -0.316, 0.]

for i in range(3):

    y[i] -= x[i]


xyDot = np.dot(x,y)
xNorm = LA.norm(x)
yNorm = LA.norm(y)

normProduct = xNorm * yNorm

if (normProduct != 0.0):

    xyDot /= normProduct

xResult = np.arccos(xyDot)

print ("Inverse cosine vals in radians:", xResult)
print ("Inverse cosine vals in degrees:", math.degrees(xResult))