import roboticstoolbox as rtb
import math
import numpy as np
from roboticstoolbox import DHRobot, RevoluteDH, PrismaticDH
import spatialmath
from spatialmath import SE3

## Create Model
# link lengths in mm
a1 = float(input("a1 = "))
a2 = float(input("a2 = "))
a3 = float(input("a3 = "))
a4 = float(input("a4 = "))

# link conversion to meter
def mm_to_meter(a):
    m = 1000 # 1 meter = 1000 mm
    return a/m

a1 = mm_to_meter(a1)
a2 = mm_to_meter(a2) 
a3 = mm_to_meter(a3) 
a4 = mm_to_meter(a4) 

# link limits converted to meter (d1)
d1 = float(input("d1 = "))
d1 = mm_to_meter(d1)

d2 = float(input("d2 = "))
d2 = mm_to_meter(d2)

d3 = float(input("d3 = "))
d3 = mm_to_meter(d3)

#Create links
#robot_variable = DHRobot([ReboluteDH(r,alpha,offset,qlim)])
CARTESIAN = DHRobot([
    PrismaticDH(0,0,(270/180.0)*np.pi,a1,qlim=[0,0]),
    PrismaticDH((270/180.0)*np.pi,0,(270/180.0)*np.pi,a2,qlim=[0,d1]),
    PrismaticDH((90/180.0)*np.pi,0,(270/180.0)*np.pi,a3,qlim=[0,d2]),
    PrismaticDH(0,0,0,a4,qlim=[0,d3]),
    
], name = "CARTESIAN")
print(CARTESIAN)

q0 = np.array([0,0,0,0])

q1 = ([0,
       mm_to_meter(float(input("d1 = "))),
       mm_to_meter(float(input("d2 = "))),
       mm_to_meter(float(input("d3 = ")))])

q2 = ([0,
       mm_to_meter(float(input("d1 = "))),
       mm_to_meter(float(input("d2 = "))),
       mm_to_meter(float(input("d3 = ")))])

q3 = ([0,
       mm_to_meter(float(input("d1 = "))),
       mm_to_meter(float(input("d2 = "))),
       mm_to_meter(float(input("d3 = ")))])

traj1 = rtb.jtraj(q0,q1,20)
traj2 = rtb.jtraj(q1,q2,20)
traj3 = rtb.jtraj(q2,q3,20)

x1 = -0.1
x2 = 0.1
y1 = -0.1
y2 = 0.1
z1 = 0.0
z2 = 0.1


CARTESIAN.plot(traj1.q,limits=[x1,x2,y1,y2,z1,z2])
CARTESIAN.plot(traj2.q,limits=[x1,x2,y1,y2,z1,z2])
CARTESIAN.plot(traj3.q,limits=[x1,x2,y1,y2,z1,z2],block=True)
