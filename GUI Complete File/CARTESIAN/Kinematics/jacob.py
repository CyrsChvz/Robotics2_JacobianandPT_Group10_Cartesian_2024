import numpy as np

# link lengths in mm
a1 = float(input("a1 = "))
a2 = float(input("a2 = "))
a3 = float(input("a3 = "))
a4 = float(input("a4 = "))

# joint variables: is mm if f, is degrees if theta
d1 = float(input("d1 = ")) #20 mm
d2 = float(input("d2 = "))
d3 = float(input("d3 = "))

# Parametric Table (theta, alpha, r, d)
PT = [[(0.0/180.0)*np.pi,(90.0/180.0)*np.pi,0,a1],
      [(90.0/180.0)*np.pi,(90.0/180.0)*np.pi,0,a2+d1],
      [(270.0/180.0)*np.pi,(90.0/180.0)*np.pi,0,a3+d2],
      [(0.0/180.0)*np.pi,(0.0/180.0)*np.pi,0,a4+d3]]

# HTM formulae
i = 0
H0_1 = [[np.cos(PT[i][0]),-np.sin(PT[i][0])*np.cos(PT[i][1]),np.sin(PT[i][0])*np.sin(PT[i][1]),PT[i][2]*np.cos(PT[i][0])],
        [np.sin(PT[i][0]),np.cos(PT[i][0])*np.cos(PT[i][1]),-np.cos(PT[i][0])*np.sin(PT[i][1]),PT[i][2]*np.sin(PT[i][0])],
        [0,np.sin(PT[i][1]),np.cos(PT[i][1]),PT[i][3]],
        [0,0,0,1]]

i = 1
H1_2 = [[np.cos(PT[i][0]),-np.sin(PT[i][0])*np.cos(PT[i][1]),np.sin(PT[i][0])*np.sin(PT[i][1]),PT[i][2]*np.cos(PT[i][0])],
        [np.sin(PT[i][0]),np.cos(PT[i][0])*np.cos(PT[i][1]),-np.cos(PT[i][0])*np.sin(PT[i][1]),PT[i][2]*np.sin(PT[i][0])],
        [0,np.sin(PT[i][1]),np.cos(PT[i][1]),PT[i][3]],
        [0,0,0,1]]

i = 2
H2_3 = [[np.cos(PT[i][0]),-np.sin(PT[i][0])*np.cos(PT[i][1]),np.sin(PT[i][0])*np.sin(PT[i][1]),PT[i][2]*np.cos(PT[i][0])],
        [np.sin(PT[i][0]),np.cos(PT[i][0])*np.cos(PT[i][1]),-np.cos(PT[i][0])*np.sin(PT[i][1]),PT[i][2]*np.sin(PT[i][0])],
        [0,np.sin(PT[i][1]),np.cos(PT[i][1]),PT[i][3]],
        [0,0,0,1]]

i = 3
H3_4 = [[np.cos(PT[i][0]),-np.sin(PT[i][0])*np.cos(PT[i][1]),np.sin(PT[i][0])*np.sin(PT[i][1]),PT[i][2]*np.cos(PT[i][0])],
        [np.sin(PT[i][0]),np.cos(PT[i][0])*np.cos(PT[i][1]),-np.cos(PT[i][0])*np.sin(PT[i][1]),PT[i][2]*np.sin(PT[i][0])],
        [0,np.sin(PT[i][1]),np.cos(PT[i][1]),PT[i][3]],
        [0,0,0,1]]

H0_1 = np.array(H0_1)
#print("H0_1= ")
#print(H0_1)

H1_2 = np.array(H1_2)
#print("H1_2= ")
#print(H1_2)

H2_3 = np.array(H2_3)
#print("H2_3= ")
#print(H2_3)

H3_4 = np.array(H3_4)
#print("H3_3= ")
#print(H3_4)

H0_2 = np.dot(H0_1,H1_2)
H0_3 = np.dot(H0_2,H2_3)
H0_4 = np.dot(H0_3,H3_4)

#print("H0_4= ")
#print(np.matrix(np.around(H0_4,3)))
#print(H0_4)

## Jacobian Matrix Program
#1. Linear/Translation Vectors
Z_1 = [[0],[0],[1]] #0,0,1 vector

    #Row 1 to 3, column 1
J1 = [[1,0,0],[0,1,0],[0,0,1]] #R0_0
J1 = np.dot(J1,Z_1)
J1 = np.matrix(J1)

    #Row 1 to 3, column 2
Z_2 = [[0],[0],[1]] #0,0,1 vector

    #Row 1 to 3, column 2
J2 = [[0,0,1],[1,0,0],[0,1,0]] #R0_1
J2 = np.dot(J2,Z_2)
J2 = np.matrix(J2)

    #Row 1 to 3, column 3
Z_3 = [[0],[0],[1]] #0,0,1 vector

    #Row 1 to 3, column 3
J3a = [[0,0,1],[1,0,0],[0,1,0]]
        
J3b = [[0,0,-1],[1,0,0],[0,-1,0]] #R0_2
        
J3c = np.dot(J3a,J3b)
J3 = np.dot(J3c,Z_3)
J3 = np.matrix(J3)

    #Row 4 to 6, column 1
J4 = [[0],[0],[0]]
J4 = np.matrix(J4)

    #Row 4 to 6, column 2
J5 = J4
J5 = np.matrix(J5)
        
    #Row 4 to 6, column 3
J6 = J4
J6 = np.matrix(J6)

    #3. Concatenated Jacobian Matrix
JM1= np.concatenate((J1,J2,J3),1)
JM2 = np.concatenate((J4,J5,J6),1)

J = np.concatenate((JM1,JM2),0)
J = np.matrix(J)
