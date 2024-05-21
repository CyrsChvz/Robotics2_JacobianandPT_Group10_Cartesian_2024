import tkinter as tk
from tkinter import *
from tkinter import messagebox, Toplevel, Button, PhotoImage
from PIL import ImageTk, Image
import pygame
import customtkinter
import numpy as np
import sympy as sp
import math
import roboticstoolbox as rtb
from roboticstoolbox import DHRobot, RevoluteDH, PrismaticDH
import spatialmath
from spatialmath import SE3
import matplotlib
from tkinter import font as tkFont
from playsound import playsound

matplotlib.use('GTK3Agg')

# Creating a GUI window with a title and an icon.
gui = Tk()
gui.title("CARTESIAN FK & IK CALCULATOR")
gui.resizable(True,True)
gui.geometry("800x500") 
gui.configure(bg="gray85",cursor='heart')

playsound('Voicy_Go Go Power Rangers (online-video-cutter.com) (1).mp3')

def f_k():

    playsound('fk.mp3')

    try:
        # link lengths in cm
        a1 = float(a1_E.get())
        a2 = float(a2_E.get())
        a3 = float(a3_E.get())
        a4 = float(a4_E.get())
        # joint variables: is cm if f, is degrees if theta
        d1 = float(d1_E.get())
        d2 = float(d2_E.get())
        d3 = float(d3_E.get())

    except ValueError:
        pop_up = Toplevel(master= gui)
        label2 = Label(pop_up, text = "Use only the appropriate syntax")
        label2.pack()
        playsound('Use only the appropriate 1.mp3')

    # Parametric Table (theta, alpha, r, d)
    PT = [[(0.0/180.0)*np.pi,(270.0/180.0)*np.pi,0,a1],
        [(270.0/180.0)*np.pi,(270.0/180.0)*np.pi,0,a2+d1],
        [(90.0/180.0)*np.pi,(270.0/180.0)*np.pi,0,a3+d2],
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

    # Position/Translation Joints
    H0_1 = np.matrix(H0_1)

    H1_2 = np.matrix(H1_2)

    H2_3 = np.matrix(H2_3)

    H3_4 = np.matrix(H3_4)

    H0_2 = np.dot(H0_1,H1_2)
    H0_3 = np.dot(H0_2,H2_3)
    H0_4 = np.dot(H0_3,H3_4)

    X0_4 = H0_4[0,3]
    X_E.delete(0,END)
    X_E.insert(0,np.around(X0_4,3))

    Y0_4 = H0_4[1,3]
    Y_E.delete(0,END)
    Y_E.insert(0,np.around(Y0_4,3))

    Z0_4 = H0_4[2,3]
    Z_E.delete(0,END)
    Z_E.insert(0,np.around(Z0_4,3))
        
    ## Jacobian Matrix Program

    # Jacobian Window

    J_sw = Toplevel()
    J_sw.title("Velocity Calculator")
    J_sw.resizable(False,False)
    J_sw.config(bg="gray85")
   
    #1. Linear/Translation Vectors
    Z_1 = [[0],
           [0],
           [1]] #0,0,1 vector

    #Row 1 to 3, column 1
    J1 = [[1,0,0],
          [0,1,0],
          [0,0,1]] #R0_0
    J1 = np.dot(J1,Z_1)
    J1 = np.matrix(J1)
    #print("J1 = ")
    #print(J1)

    #Row 1 to 3, column 2
    Z_2 = [[0],
           [0],
           [1]] #0,0,1 vector

    #Row 1 to 3, column 2
    J2 = H0_1[0:3,0:3]
    J2 = np.dot(J2,Z_2)
    J2 = np.array(J2)
    #print("J1 = ")
    #print(J2)
    
    #Row 1 to 3, column 3
    Z_3 = [[0],
           [0],
           [1]] #0,0,1 vector

    #Row 1 to 3, column 3
    J3 = H0_2[0:3,0:3]
    J3 = np.dot(J3,Z_3)
    J3 = np.array(J3)
    #print("J3 = ")
    #print(J3)

    #Row 1 to 3, column 4
    Z_4 = [[0],
           [0],
           [1]] #0,0,1 vector

    #Row 1 to 3, column 4
    J4 = H0_3[0:3,0:3]
    J4 = np.dot(J4,Z_4)
    J4 = np.array(J4)
    #print("J4 = ")
    #print(J4)
     
    #Row 4 to 6, column 1
    J5 = [[0],[0],[0]]
    J5 = np.matrix(J5)

    #Row 4 to 6, column 2
    J6 = [[0],[0],[0]]
    J6 = np.matrix(J6)
        
    #Row 4 to 6, column 3
    J7 = [[0],[0],[0]]
    J7 = np.matrix(J7)

    #Row 4 to 6, column 4
    J8 = [[0],[0],[0]]
    J8 = np.matrix(J8)

    #3. Concatenated Jacobian Matrix
    JM1 = np.concatenate((J1,J2,J3,J4),1)
    JM2 = np.concatenate((J5,J6,J7,J8),1)

    J = np.concatenate((JM1,JM2),0)
    J = np.matrix(J)
    print("J = ")
    print(J)


    def update_velo():
        d1p = d1_slider.get()
        d2p = d2_slider.get()
        d3p = d3_slider.get()
        d4p = d4_slider.get()

        q = [[d1p],[d2p],[d3p],[d4p]]
        
        E = np.dot(J,q)
        E = np.matrix(E)
        
        xp_e = E[0,0]
        x_entry.delete(0,END)
        x_entry.insert(0,str(xp_e))

        yp_e = E[1,0]
        y_entry.delete(0,END)
        y_entry.insert(0,str(yp_e))

        zp_e = E[2,0]
        z_entry.delete(0,END)
        z_entry.insert(0,str(zp_e))

        ωx_e = E[3,0]
        ωx_entry.delete(0,END)
        ωx_entry.insert(0,str(ωx_e))

        ωy_e = E[4,0]
        ωy_entry.delete(0,END)
        ωy_entry.insert(0,str(ωy_e))

        ωz_e = E[5,0]
        ωz_entry.delete(0,END)
        ωz_entry.insert(0,str(ωz_e))

    # Jacobian Sliders

    d1_velo = Label(J_sw,text=("d1* = "),font=(5),bg="gray85",fg="midnightblue") 
    d1_slider = Scale(J_sw,from_=0,to_=30,orient=HORIZONTAL,length=100,sliderlength=10,bg="gray85",fg="midnightblue")
    d1_unit = Label(J_sw,text=("cm/s"),font=(5),bg="gray85",fg="midnightblue")

    d2_velo = Label(J_sw,text=("d2* = "),font=(5),bg="gray85",fg="midnightblue") 
    d2_slider = Scale(J_sw,from_=0,to_=30,orient=HORIZONTAL,length=100,sliderlength=10,bg="gray85",fg="midnightblue")
    d2_unit = Label(J_sw,text=("cm/s"),font=(5),bg="gray85",fg="midnightblue")

    d3_velo = Label(J_sw,text=("d3* = "),font=(5),bg="gray85",fg="midnightblue") 
    d3_slider = Scale(J_sw,from_=0,to_=30,orient=HORIZONTAL,length=100,sliderlength=10,bg="gray85",fg="midnightblue")
    d3_unit = Label(J_sw,text=("cm/s"),font=(5),bg="gray85",fg="midnightblue")
    
    # For stand
    d4_velo = Label(J_sw,text=("d4* = "),font=(5),bg="gray85",fg="midnightblue") 
    d4_slider = Scale(J_sw,from_=0,to_= 0,orient=HORIZONTAL,length=100,sliderlength=10,bg="gray85",fg="midnightblue")
    d4_unit = Label(J_sw,text=("cm/s"),font=(5),bg="gray85",fg="midnightblue")

    d1_velo.grid(row=0,column=0)
    d1_slider.grid(row=0,column=1)
    d1_unit.grid(row=0,column=2)

    d2_velo.grid(row=1,column=0)
    d2_slider.grid(row=1,column=1)
    d2_unit.grid(row=1,column=2)
    np.around(d2)

    d3_velo.grid(row=2,column=0)
    d3_slider.grid(row=2,column=1)
    d3_unit.grid(row=2,column=2)

# Jacobian Entries and Labels
    x_velo = Label(J_sw,text=("x* = "),font=(5),bg="gray85",fg="midnightblue") 
    x_entry = Entry(J_sw,width=10,font=(10))
    x_unit = Label(J_sw,text=("cm/s"),font=(5),bg="gray85",fg="midnightblue")
    x_velo.grid(row=4,column=0)
    x_entry.grid(row=4,column=1)
    x_unit.grid(row=4,column=2)

    y_velo = Label(J_sw,text=("y* = "),font=(5),bg="gray85",fg="midnightblue") 
    y_entry = Entry(J_sw,width=10,font=(10))
    y_unit = Label(J_sw,text=("cm/s"),font=(5),bg="gray85",fg="midnightblue")
    y_velo.grid(row=5,column=0)
    y_entry.grid(row=5,column=1)
    y_unit.grid(row=5,column=2)

    z_velo = Label(J_sw,text=("z* = "),font=(5),bg="gray85",fg="midnightblue") 
    z_entry = Entry(J_sw,width=10,font=(10))
    z_unit = Label(J_sw,text=("cm/s"),font=(5),bg="gray85",fg="midnightblue")
    z_velo.grid(row=6,column=0)
    z_entry.grid(row=6,column=1)
    z_unit.grid(row=6,column=2)

    ωx_velo = Label(J_sw,text=("ωx = "),font=(5),bg="gray85",fg="midnightblue") 
    ωx_entry = Entry(J_sw,width=10,font=(10))
    ωx_unit = Label(J_sw,text=("rad/s"),font=(5),bg="gray85",fg="midnightblue")
    ωx_velo.grid(row=7,column=0)
    ωx_entry.grid(row=7,column=1)
    ωx_unit.grid(row=7,column=2)

    ωy_velo = Label(J_sw,text=("ωy = "),font=(5),bg="gray85",fg="midnightblue") 
    ωy_entry = Entry(J_sw,width=10,font=(10))
    ωy_unit = Label(J_sw,text=("rad/s"),font=(5),bg="gray85",fg="midnightblue")
    ωy_velo.grid(row=8,column=0)
    ωy_entry.grid(row=8,column=1)
    ωy_unit.grid(row=8,column=2)

    ωz_velo = Label(J_sw,text=("ωz = "),font=(5),bg="gray85",fg="midnightblue") 
    ωz_entry = Entry(J_sw,width=10,font=(10))
    ωz_unit = Label(J_sw,text=("rad/s"),font=(5),bg="gray85",fg="midnightblue")
    ωz_velo.grid(row=9,column=0)
    ωz_entry.grid(row=9,column=1)
    ωz_unit.grid(row=9,column=2)

    # Update Button
    update_but = Button(J_sw,text="Update",bg="green",fg="white",command=update_velo)
    update_but.grid(row=11,column=1)


    # Create Links
    CARTESIAN = DHRobot([
            PrismaticDH(0,0,(270.0/180.0)*np.pi,a1/100,qlim=[0,(50/100)]),
            PrismaticDH((270.0/180.0)*np.pi,0,(270.0/180.0)*np.pi,a2/100,qlim=[0,(d1/100)]),
            PrismaticDH((90.0/180.0)*np.pi,0,(270.0/180.0)*np.pi,a3/100,qlim=[0,(d2/100)]),
            PrismaticDH(0,0,0,a4/100,qlim=[0,(d3/100)]),
        ], name = "CARTESIAN")

    print(CARTESIAN)
    #plot joints
    q1 = np.array([0,d1/100,d2/100,d3/100])

    #plot scale
    x1 = -0.2
    x2 = 0.6
    y1 = -0.2
    y2 = 0.6
    z1 = 0.0
    z2 = 0.6

    # Plot commands
    CARTESIAN.plot(q1,limits=[x1,x2,y1,y2,z1,z2],block=True)


def rst():

    """
    It clears all the text boxes.
    """
    a1_E.delete(0, END)
    a2_E.delete(0, END)
    a3_E.delete(0, END)
    a4_E.delete(0, END)

    d1_E.delete(0, END)
    d2_E.delete(0, END)
    d3_E.delete(0, END)
    
    X_E.delete(0, END)
    Y_E.delete(0, END)
    Z_E.delete(0, END)
    playsound('reset.mp3')

def i_k():
    ### Inverse Kinematics of Cartesian

    playsound('ik.mp3')

    #link lengths in mm
    a1 = float(a1_E.get())
    a2 = float(a2_E.get())
    a3 = float(a3_E.get())
    a4 = float(a4_E.get())


    #Position Vector in mm
    x0_4 = float(X_E.get())
    y0_4 = float(Y_E.get())
    z0_4 = float(Z_E.get())

    # Solution 1
    D1 = y0_4 - a2

    # Solution 2
    D2 = x0_4 - a3

    # Solution 3
    D3 = a1 - a4 - z0_4 

    d1_E.delete(0,END)
    d1_E.insert(0,np.around(D1,3))

    d2_E.delete(0,END)
    d2_E.insert(0,np.around(D2,3))

    d3_E.delete(0,END)
    d3_E.insert(0,np.around(D3,3))

    CARTESIAN = DHRobot([
            PrismaticDH(0,0,(270.0/180.0)*np.pi,a1/100,qlim=[0,(50/100)]),
            PrismaticDH((270.0/180.0)*np.pi,0,(270.0/180.0)*np.pi,a2/100,qlim=[0,(D1/100)]),
            PrismaticDH((90.0/180.0)*np.pi,0,(270.0/180.0)*np.pi,a3/100,qlim=[0,(D2/100)]),
            PrismaticDH(0,0,0,a4/100,qlim=[0,(D3/100)]),
        ], name = "CARTESIAN")
    
    print(CARTESIAN)

    #plot joints
    q1 = np.array([0,D1/100,D2/100,D3/100])


    #plot scale
    x1 = -0.2
    x2 = 0.6
    y1 = -0.2
    y2 = 0.6
    z1 = 0.0
    z2 = 0.6    
    

    # Plot commands
    CARTESIAN.plot(q1,limits=[x1,x2,y1,y2,z1,z2],block=True)

def pap():
    a1 = 10
    a2 = 2
    a3 = 2
    a4 = 2
    CARTESIAN = DHRobot([
             PrismaticDH(0,0,(270/180.0)*np.pi,a1,qlim=[0,0]),
             PrismaticDH((270/180.0)*np.pi,0,(270/180.0)*np.pi,a2,qlim=[0,10]),
             PrismaticDH((90/180.0)*np.pi,0,(270/180.0)*np.pi,a3,qlim=[0,10]),
             PrismaticDH(0,0,0,a4,qlim=[0,10]),
    
         ], name = "CARTESIAN")
    
    q0 = np.array([0,0,0,0])    
    q1 = np.array([0,
                   2,
                   2,
                   2])
    q2 = np.array([0,
                   2,
                   2,
                   6])
    q3 = np.array([0,
                   2,
                   2,
                   2])
    q4 = np.array([0,
                   8,
                   2,
                   2])
    q5 = np.array([0,
                   8,
                   8,
                   2])
    q6 = np.array([0,
                   8,
                   8,
                   6])
    q7 = np.array([0,
                   8,
                   8,
                   2])
    q8 = np.array([0,
                   8,
                   2,
                   2])
    q9 = np.array([0,
                   2,
                   2,
                   2])


    
    traj1 = rtb.jtraj(q0,q1,15)
    traj2 = rtb.jtraj(q1,q2,15)
    traj3 = rtb.jtraj(q2,q3,15)
    traj4 = rtb.jtraj(q3,q4,15)
    traj5 = rtb.jtraj(q4,q5,15)
    traj6 = rtb.jtraj(q5,q6,15)
    traj7 = rtb.jtraj(q6,q7,15)
    traj8 = rtb.jtraj(q7,q8,15)
    traj9 = rtb.jtraj(q8,q9,15)

    x1 = -5
    x2 = 15
    y1 = -5
    y2 = 20
    z1 = 0.0
    z2 = 15


    CARTESIAN.plot(traj1.q,limits=[x1,x2,y1,y2,z1,z2])
    CARTESIAN.plot(traj2.q,limits=[x1,x2,y1,y2,z1,z2])
    CARTESIAN.plot(traj3.q,limits=[x1,x2,y1,y2,z1,z2])
    CARTESIAN.plot(traj4.q,limits=[x1,x2,y1,y2,z1,z2])
    CARTESIAN.plot(traj5.q,limits=[x1,x2,y1,y2,z1,z2])
    CARTESIAN.plot(traj6.q,limits=[x1,x2,y1,y2,z1,z2])
    CARTESIAN.plot(traj7.q,limits=[x1,x2,y1,y2,z1,z2])
    CARTESIAN.plot(traj8.q,limits=[x1,x2,y1,y2,z1,z2])
    CARTESIAN.plot(traj9.q,limits=[x1,x2,y1,y2,z1,z2],block=True)

def wld():
    a1 = 10
    a2 = 2
    a3 = 2
    a4 = 2
    CARTESIAN = DHRobot([
             PrismaticDH(0,0,(270/180.0)*np.pi,a1,qlim=[0,0]),
             PrismaticDH((270/180.0)*np.pi,0,(270/180.0)*np.pi,a2,qlim=[0,10]),
             PrismaticDH((90/180.0)*np.pi,0,(270/180.0)*np.pi,a3,qlim=[0,10]),
             PrismaticDH(0,0,0,a4,qlim=[0,10]),
    
         ], name = "CARTESIAN")
    
    q0 = np.array([0,0,0,0])    
    q1 = np.array([0,
                   2,
                   2,
                   2])
    q2 = np.array([0,
                   2,
                   2,
                   6])
    q3 = np.array([0,
                   8,
                   2,
                   6])
    q4 = np.array([0,
                   8,
                   8,
                   6])
    q5 = np.array([0,
                   2,
                   8,
                   6])
    q6 = np.array([0,
                   2,
                   2,
                   6])
    q7 = np.array([0,
                   2,
                   2,
                   2])
    
   
    
    traj1 = rtb.jtraj(q0,q1,15)
    traj2 = rtb.jtraj(q1,q2,15)
    traj3 = rtb.jtraj(q2,q3,15)
    traj4 = rtb.jtraj(q3,q4,15)
    traj5 = rtb.jtraj(q4,q5,15)
    traj6 = rtb.jtraj(q5,q6,15)
    traj7 = rtb.jtraj(q6,q7,15)

    x1 = -5
    x2 = 15
    y1 = -5
    y2 = 20
    z1 = 0.0
    z2 = 15


    CARTESIAN.plot(traj1.q,limits=[x1,x2,y1,y2,z1,z2])
    CARTESIAN.plot(traj2.q,limits=[x1,x2,y1,y2,z1,z2])
    CARTESIAN.plot(traj3.q,limits=[x1,x2,y1,y2,z1,z2])
    CARTESIAN.plot(traj4.q,limits=[x1,x2,y1,y2,z1,z2])
    CARTESIAN.plot(traj5.q,limits=[x1,x2,y1,y2,z1,z2])
    CARTESIAN.plot(traj6.q,limits=[x1,x2,y1,y2,z1,z2])
    CARTESIAN.plot(traj7.q,limits=[x1,x2,y1,y2,z1,z2],block=True)  

def ptt():
    pt = Toplevel()
    pt.title("Path and Trajectory")
    pt.resizable(False,False)
    pt.config(bg="gray85")
    
    def update_PT():

        ## Create Model
        # link lengths in mm
        a1 = float(pa1_E.get())
        a2 = float(pa2_E.get())
        a3 = float(pa3_E.get())
        a4 = float(pa4_E.get())
    
        # link conversion to meter
        def mm_to_meter(a):
            m = 1000 # 1 meter = 1000 mm
            return a/m
    
        a1 = mm_to_meter(a1)
        a2 = mm_to_meter(a2) 
        a3 = mm_to_meter(a3) 
        a4 = mm_to_meter(a4) 
    
        # link limits converted to meter (d1)
        d1 = float(pd1_E.get())
        d1 = mm_to_meter(d1)
    
        d2 = float(pd2_E.get())
        d2 = mm_to_meter(d2)
    
        d3 = float(pd3_E.get())
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
               mm_to_meter(float(q1d1_E.get())),
               mm_to_meter(float(q1d2_E.get())),
               mm_to_meter(float(q1d3_E.get()))])
    
        q2 = ([0,
               mm_to_meter(float(q2d1_E.get())),
               mm_to_meter(float(q2d2_E.get())),
               mm_to_meter(float(q2d3_E.get()))])
    
        q3 = ([0,
               mm_to_meter(float(q3d1_E.get())),
               mm_to_meter(float(q3d2_E.get())),
               mm_to_meter(float(q3d3_E.get()))])
    
        traj1 = rtb.jtraj(q0,q1,20)
        traj2 = rtb.jtraj(q1,q2,20)
        traj3 = rtb.jtraj(q2,q3,20)
    
        x1 = -0.1
        x2 = 0.1
        y1 = -0.1
        y2 = 0.1
        z1 = 0.0
        z2 = 0.1
    
        CARTESIAN.plot(traj1.s,limits=[x1,x2,y1,y2,z1,z2])
        CARTESIAN.plot(traj2.s,limits=[x1,x2,y1,y2,z1,z2])
        CARTESIAN.plot(traj3.s,limits=[x1,x2,y1,y2,z1,z2],block=True)
    

    ptp = LabelFrame(pt,text="           Link Lengths and Joint Variables",font=("Malgun Gothic",12),bg="gray85",fg="midnightblue")
    ptp.grid(row=0,column=0)


    # The code below is creating a label frame and 
    # then creating labels and entries for the link lengths.
    # Link lengths
    pa1 = Label(ptp,text=("a1 = "),font=(10),bg="gray85",fg="midnightblue")
    pa1_E = Entry(ptp,width=10,font=(10))
    pcm1 = Label(ptp,text=("cm"),font=(10),bg="gray85",fg="midnightblue")

    pa2 = Label(ptp,text=("a2 = "),font=(10),bg="gray85",fg="midnightblue")
    pa2_E = Entry(ptp,width=10,font=(10))
    pcm2 = Label(ptp,text=("cm"),font=(10),bg="gray85",fg="midnightblue")

    pa3 = Label(ptp,text=("a3 = "),font=(10),bg="gray85",fg="midnightblue")
    pa3_E = Entry(ptp,width=10,font=(10))
    pcm3 = Label(ptp,text=("cm"),font=(10),bg="gray85",fg="midnightblue")

    pa4 = Label(ptp,text=("a4 = "),font=(10),bg="gray85",fg="midnightblue")
    pa4_E = Entry(ptp,width=10,font=(10))
    pcm4 = Label(ptp,text=("cm"),font=(10),bg="gray85",fg="midnightblue")

    pa1.grid(row=0,column=0)
    pa1_E.grid(row=0,column=1)
    pcm1.grid(row=0,column=2)

    pa2.grid(row=1,column=0)
    pa2_E.grid(row=1,column=1)
    pcm2.grid(row=1,column=2)

    pa3.grid(row=2,column=0)
    pa3_E.grid(row=2,column=1)
    pcm3.grid(row=2,column=2)

    pa4.grid(row=3,column=0)
    pa4_E.grid(row=3,column=1)
    pcm4.grid(row=3,column=2)

    # The code below is creating a label frame and 
    # then creating labels and entries for the Joint Variables.
    # Joint Variables
    pd1 = Label(ptp,text=("d1 = "),font=(10),bg="gray85",fg="midnightblue")
    pd1_E = Entry(ptp,width=10,font=(10))
    pcm7 = Label(ptp,text=("cm"),font=(10),bg="gray85",fg="midnightblue")

    pd2 = Label(ptp,text=("d2 = "),font=(10),bg="gray85",fg="midnightblue")
    pd2_E = Entry(ptp,width=10,font=(10))
    pcm8 = Label(ptp,text=("cm"),font=(10),bg="gray85",fg="midnightblue")

    pd3 = Label(ptp,text=("d3 = "),font=(10),bg="gray85",fg="midnightblue")
    pd3_E = Entry(ptp,width=10,font=(10))
    pcm9 = Label(ptp,text=("cm"),font=(10),bg="gray85",fg="midnightblue")

    pd1.grid(row=0,column=3)
    pd1_E.grid(row=0,column=4)
    pcm7.grid(row=0,column=5)

    pd2.grid(row=1,column=3)
    pd2_E.grid(row=1,column=4)
    pcm8.grid(row=1,column=5)

    pd3.grid(row=2,column=3)
    pd3_E.grid(row=2,column=4)
    pcm9.grid(row=2,column=5)

    q1d1 = Label(ptp,text=("q1_d1 = "),font=(10),bg="gray85",fg="midnightblue")
    q1d1_E = Entry(ptp,width=10,font=(10))
    cm7 = Label(ptp,text=("cm"),font=(10),bg="gray85",fg="midnightblue")

    q1d2 = Label(ptp,text=("q1_d2 = "),font=(10),bg="gray85",fg="midnightblue")
    q1d2_E = Entry(ptp,width=10,font=(10))
    cm8 = Label(ptp,text=("cm"),font=(10),bg="gray85",fg="midnightblue")

    q1d3 = Label(ptp,text=("q1_d3 = "),font=(10),bg="gray85",fg="midnightblue")
    q1d3_E = Entry(ptp,width=10,font=(10))
    cm9 = Label(ptp,text=("cm"),font=(10),bg="gray85",fg="midnightblue")

    q1d1.grid(row=3,column=4)
    q1d1_E.grid(row=3,column=5)
    cm7.grid(row=3,column=6)

    q1d2.grid(row=4,column=4)
    q1d2_E.grid(row=4,column=5)
    cm8.grid(row=4,column=6)

    q1d3.grid(row=5,column=4)
    q1d3_E.grid(row=5,column=5)
    cm9.grid(row=5,column=6)

    q2d1 = Label(ptp,text=("q2_d1 = "),font=(10),bg="gray85",fg="midnightblue")
    q2d1_E = Entry(ptp,width=10,font=(10))
    cm7 = Label(ptp,text=("cm"),font=(10),bg="gray85",fg="midnightblue")

    q2d2 = Label(ptp,text=("q2_d2 = "),font=(10),bg="gray85",fg="midnightblue")
    q2d2_E = Entry(ptp,width=10,font=(10))
    cm8 = Label(ptp,text=("cm"),font=(10),bg="gray85",fg="midnightblue")

    q2d3 = Label(ptp,text=("q2_d3 = "),font=(10),bg="gray85",fg="midnightblue")
    q2d3_E = Entry(ptp,width=10,font=(10))
    cm9 = Label(ptp,text=("cm"),font=(10),bg="gray85",fg="midnightblue")

    q2d1.grid(row=6,column=4)
    q2d1_E.grid(row=6,column=5)
    cm7.grid(row=6,column=6)

    q2d2.grid(row=7,column=4)
    q2d2_E.grid(row=7,column=5)
    cm8.grid(row=7,column=6)

    q2d3.grid(row=8,column=4)
    q2d3_E.grid(row=8,column=5)
    cm9.grid(row=8,column=6)

    q3d1 = Label(ptp,text=("q3_d1 = "),font=(10),bg="gray85",fg="midnightblue")
    q3d1_E = Entry(ptp,width=10,font=(10))
    cm7 = Label(ptp,text=("cm"),font=(10),bg="gray85",fg="midnightblue")

    q3d2 = Label(ptp,text=("q3_d2 = "),font=(10),bg="gray85",fg="midnightblue")
    q3d2_E = Entry(ptp,width=10,font=(10))
    cm8 = Label(ptp,text=("cm"),font=(10),bg="gray85",fg="midnightblue")

    q3d3 = Label(ptp,text=("q3_d3 = "),font=(10),bg="gray85",fg="midnightblue")
    q3d3_E = Entry(ptp,width=10,font=(10))
    cm9 = Label(ptp,text=("cm"),font=(10),bg="gray85",fg="midnightblue")

    q3d1.grid(row=9,column=4)
    q3d1_E.grid(row=9,column=5)
    cm7.grid(row=9,column=6)

    q3d2.grid(row=10,column=4)
    q3d2_E.grid(row=10,column=5)
    cm8.grid(row=10,column=6)

    q3d3.grid(row=11,column=4)
    q3d3_E.grid(row=11,column=5)
    cm9.grid(row=11,column=6)

    update_pt = Button(ptp,text="Update",bg="green",fg="white",command=update_PT)
    update_pt.grid(row=9,column=1)

#path and trajectory planning

PTP = LabelFrame(gui,text="Path and Trajectory Planning",font=(12),bg="gray85",fg="midnightblue")
PTP.grid(row=5,column=0)

PT = Button(PTP, text="         Press for Demo         ",font=(10),bg="Yellow",fg="black",command=ptt)
PT.grid(row=0, column=0)

# The code below is creating a label frame, which is a frame that contains labels. The label frame is called PV, and it is placed in the gui window. The label frame is placed in the second row and the first column.

# Position Vectors Frame
PV = LabelFrame(gui,text="   Position Vectors  ",font=(12),bg="gray85",fg="midnightblue")
PV.grid(row=3,column=0)

# Position Vector
X = Label(PV,text=("X = "),font=(10),bg="gray85",fg="midnightblue")
X_E = Entry(PV,width=10,font=(10))
cm10 = Label(PV,text=("cm"),font=(10),bg="gray85",fg="midnightblue")

Y = Label(PV,text=("Y = "),font=(10),bg="gray85",fg="midnightblue")
Y_E = Entry(PV,width=10,font=(10))
cm11 = Label(PV,text=("cm"),font=(10),bg="gray85",fg="midnightblue")

Z = Label(PV,text=("Z = "),font=(10),bg="gray85",fg="midnightblue")
Z_E = Entry(PV,width=10,font=(10))
cm12 = Label(PV,text=("cm"),font=(10),bg="gray85",fg="midnightblue")

X.grid(row=0,column=0)
X_E.grid(row=0,column=1)
cm10.grid(row=0,column=2)

Y.grid(row=1,column=0)
Y_E.grid(row=1,column=1)
cm11.grid(row=1,column=2)

Z.grid(row=2,column=0)
Z_E.grid(row=2,column=1)
cm12.grid(row=2,column=2)

def Quit():
  answer = messagebox.askokcancel('Quit', '   Are you sure?')
  if answer:
     gui.destroy()

Q = LabelFrame(gui,text="Press when done",font=(12),bg="gray85",fg="midnightblue")
Q.grid(row=7,column=0)

QIT = Button(Q, text="        Quit         ",font=(10),bg="black",fg="gray85",command=Quit)
QIT.grid(row=0, column=0)

f = LabelFrame(gui,text="",font=(12),bg="gray85",fg="midnightblue")
f.grid(row=6,column=0)     
rb = Button(f, text="The best way to predict the future is to invent it",font=(10),bg="gray85",fg="black")
Weld = Button(f,text="  Welding  ",font=(10),bg="violet",fg="gray85",command=wld)
pnp = Button(f,text="  Pick And Place  ",font=(10),bg="violet",fg="gray85",command=pap)

Weld.grid(row=2, column=0)
pnp.grid(row=3, column=0)
rb.grid(row=0, column=0)

# Link Lengths and Joint Variables Frame
    
FI = LabelFrame(gui,text="           Link Lengths and Joint Variables",font=("Malgun Gothic",12),bg="gray85",fg="midnightblue")
FI.grid(row=0,column=0)


# The code below is creating a label frame and 
# then creating labels and entries for the link lengths.
# Link lengths
a1 = Label(FI,text=("a1 = "),font=(10),bg="gray85",fg="midnightblue")
a1_E = Entry(FI,width=10,font=(10))
cm1 = Label(FI,text=("cm"),font=(10),bg="gray85",fg="midnightblue")

a2 = Label(FI,text=("a2 = "),font=(10),bg="gray85",fg="midnightblue")
a2_E = Entry(FI,width=10,font=(10))
cm2 = Label(FI,text=("cm"),font=(10),bg="gray85",fg="midnightblue")

a3 = Label(FI,text=("a3 = "),font=(10),bg="gray85",fg="midnightblue")
a3_E = Entry(FI,width=10,font=(10))
cm3 = Label(FI,text=("cm"),font=(10),bg="gray85",fg="midnightblue")

a4 = Label(FI,text=("a4 = "),font=(10),bg="gray85",fg="midnightblue")
a4_E = Entry(FI,width=10,font=(10))
cm4 = Label(FI,text=("cm"),font=(10),bg="gray85",fg="midnightblue")

a1.grid(row=0,column=0)
a1_E.grid(row=0,column=1)
cm1.grid(row=0,column=2)

a2.grid(row=1,column=0)
a2_E.grid(row=1,column=1)
cm2.grid(row=1,column=2)

a3.grid(row=2,column=0)
a3_E.grid(row=2,column=1)
cm3.grid(row=2,column=2)

a4.grid(row=3,column=0)
a4_E.grid(row=3,column=1)
cm4.grid(row=3,column=2)

# The code below is creating a label frame and 
# then creating labels and entries for the Joint Variables.
# Joint Variables
d1 = Label(FI,text=("d1 = "),font=(10),bg="gray85",fg="midnightblue")
d1_E = Entry(FI,width=10,font=(10))
cm7 = Label(FI,text=("cm"),font=(10),bg="gray85",fg="midnightblue")

d2 = Label(FI,text=("d2 = "),font=(10),bg="gray85",fg="midnightblue")
d2_E = Entry(FI,width=10,font=(10))
cm8 = Label(FI,text=("cm"),font=(10),bg="gray85",fg="midnightblue")

d3 = Label(FI,text=("d3 = "),font=(10),bg="gray85",fg="midnightblue")
d3_E = Entry(FI,width=10,font=(10))
cm9 = Label(FI,text=("cm"),font=(10),bg="gray85",fg="midnightblue")

d1.grid(row=0,column=3)
d1_E.grid(row=0,column=4)
cm7.grid(row=0,column=5)

d2.grid(row=1,column=3)
d2_E.grid(row=1,column=4)
cm8.grid(row=1,column=5)

d3.grid(row=2,column=3)
d3_E.grid(row=2,column=4)
cm9.grid(row=2,column=5)

# The code below is creating a frame and buttons.
# Buttons Frame
BF = LabelFrame(gui,text="       Forward & Inverse Kinematics     ",font=(12),bg="gray85",fg="midnightblue")
BF.grid(row=1,column=0)

# Buttons
FK = Button(BF,text="↓ Forward",font=(10),bg="green",fg="gray85",command=f_k)
rst = Button(BF,text="  Reset  ",font=(10),bg="red",fg="gray85",command=rst)
IK = Button(BF,text="↑ Inverse",font=(10),bg="blue",fg="gray85",command=i_k)

FK.grid(row=0, column=0)
rst.grid(row=0, column=1)
IK.grid(row=0, column=2)

#path and trajectory planning

PTP = LabelFrame(gui,text="Path and Trajectory Planning",font=(12),bg="gray85",fg="midnightblue")
PTP.grid(row=5,column=0)

PT = Button(PTP, text="         Press for Demo         ",font=(10),bg="Yellow",fg="black",command=ptt)
PT.grid(row=0, column=0)

# The code below is creating a label frame, which is a frame that contains labels. The label frame is called PV, and it is placed in the gui window. The label frame is placed in the second row and the first column.

# Position Vectors Frame
PV = LabelFrame(gui,text="   Position Vectors  ",font=(12),bg="gray85",fg="midnightblue")
PV.grid(row=3,column=0)

# Position Vector
X = Label(PV,text=("X = "),font=(10),bg="gray85",fg="midnightblue")
X_E = Entry(PV,width=10,font=(10))
cm10 = Label(PV,text=("cm"),font=(10),bg="gray85",fg="midnightblue")

Y = Label(PV,text=("Y = "),font=(10),bg="gray85",fg="midnightblue")
Y_E = Entry(PV,width=10,font=(10))
cm11 = Label(PV,text=("cm"),font=(10),bg="gray85",fg="midnightblue")

Z = Label(PV,text=("Z = "),font=(10),bg="gray85",fg="midnightblue")
Z_E = Entry(PV,width=10,font=(10))
cm12 = Label(PV,text=("cm"),font=(10),bg="gray85",fg="midnightblue")

X.grid(row=0,column=0)
X_E.grid(row=0,column=1)
cm10.grid(row=0,column=2)

Y.grid(row=1,column=0)
Y_E.grid(row=1,column=1)
cm11.grid(row=1,column=2)

Z.grid(row=2,column=0)
Z_E.grid(row=2,column=1)
cm12.grid(row=2,column=2)

# image frame

cv1 = Canvas()
cv1.place(x=410, y= 190)

model1 = ImageTk.PhotoImage(Image.open("cart.png").resize((380,290)))
img_1 = Label(cv1, image=model1)
img_1.pack(fill="none")

cv2 = Canvas()
cv2.place(x= 400, y= 0)

model2 = ImageTk.PhotoImage(Image.open("bg.png").resize((400,200)))
img_2 = Label(cv2, image=model2)
img_2.pack(fill="none")

gui.mainloop()
