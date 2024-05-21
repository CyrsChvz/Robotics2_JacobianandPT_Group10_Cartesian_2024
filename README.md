<a name="readme-top"> </a>
## Please download the whole "GUI Complete File" folder and its content for the program to run properly in Python, Thank you!
ps. Open the folder "GUI Complete File", and make sure that all the contents of the "GUI Complete File" are downloaded and compiled in the same folder for the program to run properly. In addition!!! please refer to the "blue texts" for easier travel throughout the readme file. Again, thank you!

<div align="center">
    
![XSIZnr](https://github.com/CyrsChvz/Robotics2_FK-IK_Group10_CartesianManipulator_2024/assets/157597327/80993792-9725-4d53-af5e-9835d461c6fc)
![XSIZnr](https://github.com/CyrsChvz/Robotics2_FK-IK_Group10_CartesianManipulator_2024/assets/157597327/80993792-9725-4d53-af5e-9835d461c6fc)

</div>

![GitHub forks](https://img.shields.io/github/forks/CyrsChvz/Robotics2_FK-IK_Group10_CartesianManipulator_2024?style=for-the-badge&logo=github&logoColor=%23ff0000)
![GitHub forks](https://img.shields.io/github/stars/CyrsChvz/Robotics2_FK-IK_Group10_CartesianManipulator_2024?style=for-the-badge&logo=github&logoColor=%23ff0000)
![GitHub Issues or Pull Requests](https://img.shields.io/github/issues/CyrsChvz/Robotics2_FK-IK_Group10_CartesianManipulator_2024?style=for-the-badge&logo=github&logoColor=%23ff0000)
![GitHub watchers](https://img.shields.io/github/watchers/CyrsChvz/Robotics2_FK-IK_Group10_CartesianManipulator_2024?style=for-the-badge&logo=github&logoColor=%23ff0000)
![GitHub commit activity](https://img.shields.io/github/commit-activity/w/CyrsChvz/Robotics2_FK-IK_Group10_CartesianManipulator_2024?style=for-the-badge&logo=github&logoColor=%23ff0000)


<!-- PROJECT LOGO -->
<br />
<div align="center">
    <img alt="large" width="560" height="400" src="https://github.com/CyrsChvz/Robotics2_FK-IK_Group10_CartesianManipulator_2024/assets/157597327/bb7dc30c-e405-4a04-a016-4c8c5963a84f"
    <a href="https://github.com/cyrschavz/Robotics2_FK-IK_Group10_CartesianManipulator_2024">
<h3 align="center">BreadcrumbsRobotics2_JacobianandPT_Group10_Cartesian_2024</h3>

  <p align="center">
    Project_Description
    <br />
    <a href="https://drive.google.com/file/d/1h_7uCnrbAoJrcapAplV_65SiYqNWTWeF/view?usp=sharing"><strong> Watch a video encompassing the contents of a Cartesian Manipulator »</strong></a>
    <br />
    <br />

  </p>
</div>


<!-- TABLE OF CONTENTS -->
<details>
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="#Abstract of the Project">Abstract of the Project</a>
    </li>
    <li>
      <a href="#Introduction of the Project">Introduction of the Project</a>
    </li>

<li><a href="#Jacobian Matrix">Jacobian Matrix.</a></li>
    <li><a href="#Differential Equations">Differential Equations.</a></li>
    <li><a href="#Singularities">Singularities.</a></li>
    <li><a href="#Path and Trajectory Planning Utilizing GUI">Path and Trajectory Planning Utilizing GUI.</a></li>
  
  </ol>
</details>



<a name="Abstract of the Project"> </a>
# I. Abstract of the Project
<div align="justify">
  
This project investigates the basic ideas and real-world uses of a Cartesian manipulator, with an emphasis on their kinematics from a variety of angles, such as degrees of freedom (DOF), Denavit-Hartenberg (D-H) notation, Homogeneous Transformation matrices, parametric table, as well as the opposite kinematics. The research starts with an extensive:

</div>

<a href="#Jacobian Matrix">`Jacobian Matrix`</a>

</div>
<div>
    
<a href="#Differential Equations">`Differential Equations`</a>    

</div>
<div>

<a href="#Singularities">`Singularities`</a>  
 
</div>
<div>

<a href="#Path and Trajectory Planning Utilizing GUI">`Path and Trajectory Planning Utilizing GUI`</a>  
 
</div>
<div>



<p align="right">(<a href="#readme-top">back to top</a>)</p>


<a name="Introduction of the Project"> </a>
# II. Introduction of the Project
<div align="justify">
  
The successful control and analysis of Cartesian manipulators, also known as robotic arms, hinge on several key concepts. The Jacobian matrix acts as a translator, connecting the velocities of the manipulator's joints (joint space) to the movements of its end-effector (Cartesian space). This allows us to understand how joint rotations translate into the end-effector's movements and speeds. Differential equations, on the other hand, describe the relationship between the manipulator's state (joint positions and velocities) at any given moment and how those states change over time. Solving these equations enables us to predict and simulate the manipulator's motion.

However, there are specific configurations where the Jacobian matrix loses its effectiveness. These are called singularities, and they limit the manipulator's maneuverability and can cause control problems. Identifying and avoiding singularities is crucial for safe and efficient operation. To achieve smooth and controlled movements, we need to plan the path the end-effector will follow, considering obstacles and the manipulator's workspace limitations. This process, called path planning, is followed by trajectory planning, which adds timing information (velocities and accelerations) to each point along the path.

Finally, Graphical User Interfaces (GUIs) can be created using tools like MATLAB's GUIDE or Python libraries. These GUIs allow us to visualize the manipulator, manipulate its joints through sliders, and dynamically update the path or trajectory based on user input. This provides a user-friendly platform for controlling and monitoring the manipulator's motion.
  
</div>

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<a name="Jacobian Matrix"> </a>
# III. Jacobian Matrix
<div align="justify">
  
To solve a DOF of a specific manipulator the first thing to do is to determine whether it is a spatial with 6 DOF or planar with 3 DOF. The next step is to figure out the number of joints and moving links on the manipulator. After that, the calculation of the number of joint constraints in the given manipulator and determining if it is spatial or planar with the help of Grubler’s Criterion. Lastly, determine the type of manipulator based on the number of degrees of freedom. To calculate the degrees of freedom of the Cartesian Manipulator, use Grubler's Formula. This is an example of how to list things you need to use the software and how to install them.

</div>

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<a name="Differential Equations"> </a>
# IV. Differential Equations
<div align="justify">
    
 The Denavit-Hartenberg Notation, often known as D-H Notation, was developed in 1995 by Jacques Denavit and Richard Hartenberg to standardize coordinate frames for spatial links.

 To solve the forward kinematics of a mechanical manipulator we will use the DH Notation (Denavit-Hartenberg Notation)
 The D-H notation offers a systematic approach to express the geometric configuration of robotic systems, making kinematic analysis and modeling easier.
 It is frequently used in robotics, particularly industrial robot systems, and robot arms with manipulators.
 In DH notation, there are some preliminary rules and main rules that define how to assign coordinate frames and determine the parameters for each joint.








 ![rgb](https://github.com/CyrsChvz/Robotics2_FK-IK_Group10_CartesianManipulator_2024/assets/157597327/81ea8b50-e43f-41f6-b446-9cb2dab1c1d4)
 
    
    
    



    The y-axis is less important than the x and z axes

## D-H Frame Rules

### NOTE: THE COUNTING OF FRAMES STARTS FROM 0 (FROM THE FORMULA N-1)
    
    
    
    
![zaxis](https://github.com/CyrsChvz/Robotics2_FK-IK_Group10_CartesianManipulator_2024/assets/157597327/4b7ff0d5-2072-4309-8dda-53d5e8b4b87c)

    Rule 2: The X axis must be perpendicular both to its own Z axis, and the 
    Z axis of the frame before it. (Labels starts from X0)
![xaxis](https://github.com/CyrsChvz/Robotics2_FK-IK_Group10_CartesianManipulator_2024/assets/157597327/87e45ec4-e768-494c-834f-1bfa41e29ffb)

    Rule 3: Each X-axis must intersect the Z-axis of the frame before it. 
    Rules for complying with Rule 3: Rotate the axis until it hits the other.
    Or translate the axis until it hits the other.
![xaxis](https://github.com/CyrsChvz/Robotics2_FK-IK_Group10_CartesianManipulator_2024/assets/157597327/87e45ec4-e768-494c-834f-1bfa41e29ffb)

    Rule 4: All frames must follow the right-hand rule (Labels starts from Y0)
   ![dh](https://github.com/CyrsChvz/Robotics2_FK-IK_Group10_CartesianManipulator_2024/assets/157597327/16d5305d-c432-452e-a01e-d1beb04f42d7)
 
</div>

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<a name="Singularities"> </a>
# V. Singularities
<div align="justify">
    
The DH parametric table is like a blueprint for robotic arms. It helps engineers understand how the parts of the arm fit together and move, making it easier to design and control the robot. Using the Denavit Hartenberg Parameters we would be able to create a Dh parametric Table for a Cartesian Manipulator

</div>

### The Denavit Hartenberg Parameters as it follows :

<div align="left">

    Theta (θ) - Rotation around Zn-1 that is required to get Xn-1 to match Xn, with the joint 
    variable, if joint is revolute/twisting jont.
    
    Alpha (α) - Rotation around Xn that is required to get Zn-1 to match Zn.
    
    d - The distance from the origin of n-1 and n frames along the Zn-1 direction, 
    with a joint variable if joint is prismatic.
    
    r - The distance from the origin of n-1 and n frames along the Xn direction.

</div>

<div align="justify">
    
### In filling up the Dh parametric table of the cartesian manipulator  start with filling out the row of theta, then alpha, after that is the column of r and lastly the columns of d.

</div>

<div align="center">

    |n   |   θ   |    ∝   |  r  |     d     |
    |1   |   0   |   270  |  0  |     a1    |
    |2   |  270  |   270  |  0  |   a2+d1   |
    |3   |  90   |   270  |  0  |   a3+d2   |
    |4   |   0   |    0   |  0  |   a4+d3   |


</div>

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<a name="Path and Trajectory Planning Utilizing GUI"> </a>
# VI. Path and Trajectory Planning Utilizing GUI
<div align="justify">

In a treasure hunt analogy, a Hierarchical Task Manager (HTM) acts like a special code guiding a friend to hidden loot. No matter the starting point in the room, the HTM provides clear instructions in two parts:
     
Direction: This part tells the friend which way to turn (left, right, up, down) to face the treasure chest.
Steps: Once facing the right direction, the code specifies how many steps to take to reach the treasure.

Now, imagine a robotic arm replacing the treasure in the room. HTMs function similarly but in a 3D space, instructing the robot's computer on two key actions:
Rotate the Arm: This is akin to pointing a "finger" in the right direction, just like aiming for the treasure.
Extend the Arm: The code also dictates how far the robot's arm needs to extend (like taking steps) to reach and grasp an object. 

By combining these instructions (pointing and reaching) into a single code (HTM), the robot's computer can precisely control where the arm goes
This simplifies understanding and controlling even complex robot arms with many joints, ensuring they reach their designated positions accurately. 

Homogeneous transformation matrices enable us to combine rotation matrices (which have 3 rows and 3 columns) and displacement 
vectors (which have 3 rows and 1 column) into a single matrix. They are an important concept of forward Kinematics.

## Here's a breakdown of how to solve for the homogeneous transformation matrix (HTM) of a Cartesian manipulator:

### 1. Leverage the Denavit-Hartenberg (DH) Convention: The DH convention provides a systematic way to define the transformations between each joint in a manipulator. It uses four parameters for each joint:
      
      α (alpha) which si the value of Twists in an angle between the previous and current z-axes.
      a : Offset distance along the previous z-axis.
      θ (theta) which is the Rotation angle about the current z-axis.
      d (d) as the Offset distance along the current x-axis.

### 2. Identify the Joint Types:
Determine the type of joint at each connection point (usually revolute or prismatic). This helps define the appropriate rotation or translation for each transformation.   

### 3. Build Individual Transformation Matrices

For each joint, create a 4x4 HTM using its DH parameters. The general form of the matrix is:

    | cos(θ)  -sin(θ)*cos(α)  sin(θ)*sin(α)  a*cos(θ) |
    | sin(θ)   cos(θ)*cos(α) -cos(θ)*sin(α)  a*sin(θ) |
    |  0          sin(α)          cos(α)        d     |
    |  0            0               0           1     |

Substitute the values on the parametric table you have taken,
the respective rows on the parametric table and their contents will be used for the HTM rows they are in line with,
substituting the angles gained from the theta, alpha, rotation and distance to the equation matrix used for finding the HTM.

### 4. Multiplying the Transformation Matrices of each frame you had come up with by chaining the HTM”s. This can be done by multiplying them together in the order the joints appear in the manipulator arm, starting from the base and moving towards the end effector. This gives you the final HTM that describes the position and orientation of the end effector relative to the base frame.

</div>

<div align="center">
    
    |  cos(0)   -sin(0)*cos(270)   sin(0)*sin(270)   0*cos(0) |
    |  sin(0)    cos(0)*cos(270)  -cos(0)*sin(270)   0*sin(0) |
    |    0          sin(270)           cos(α)          a1     |
    |    0             0                 0              1     |

    | cos(270)  -sin(270)*cos(0)  sin(270)*sin(0)  0*cos(270) |
    | sin(270)   cos(270)*cos(0) -cos(270)*sin(0)  a*sin(270) |
    |    0          sin(0)            cos(0)        a2 + d1   |
    |    0            0                 0              1      |

    | cos(90)   -sin(90)*cos(270)   sin(θ)*sin(90)  0*cos(90) |
    | sin(90)    cos(90)*cos(90)   -cos(θ)*sin(90)  0*sin(90) |
    |    0          sin(270)           cos(90)       a3 + d2  |
    |    0             0                 0              1     |

    |  cos(0)    -sin(0)*cos(0)    sin(0)*sin(0)    0*cos(0)  |
    |  sin(0)     cos(0)*cos(0)   -cos(0)*sin(0)    0*sin(90) |
    |    0            sin(α)          cos(α)         a4 +d3   |
    |    0              0               0              1      |
     
![htm](https://github.com/CyrsChvz/Robotics2_FK-IK_Group10_CartesianManipulator_2024/assets/157597327/0623c414-ca8a-4ba3-8470-85a3016b8bc9)

</div>



<p align="right">(<a href="#readme-top">back to top</a>)</p>


# |References|



<p align="right">(<a href="#readme-top">back to top</a>)</p>
