from se2 import *
import numpy as np

a = SE2((0,0), 0) # Origin Frame
b = SE2((5,3), np.pi/2) # Frame A
c = b*a# Frame A to O
p = [1,2] # point O
d = c.inv()*p # Point D in A's perspective
print(c)
print(d) # prints point D in A's perspective
print(getAngle(d))