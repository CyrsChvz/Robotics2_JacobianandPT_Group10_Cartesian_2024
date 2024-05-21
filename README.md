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
  
Coordinating a robotic arm with multiple joints requires understanding the relationship between how fast each joint moves (joint velocities) and the overall movement of the gripper or tool at the end (end-effector velocities). The Jacobian Matrix acts as a translator between these two worlds: joint space and Cartesian space (real-world space).

This matrix takes a vector representing the velocities of each joint and transforms it into a vector representing the end-effector's velocities (both linear, like speed along an axis, and angular, like rotation). Each element in the Jacobian Matrix reflects how a specific joint's speed contributes to a particular component of the end-effector's movement. This knowledge is crucial for several reasons:

-This matrix takes a vector representing the velocities of each joint and transforms it into a vector representing the end-effector's velocities (both linear, like speed along an axis, and angular, like rotation). Each element in the Jacobian Matrix reflects how a specific joint's speed contributes to a particular component of the end-effector's movement. This knowledge is crucial for several reasons:

-The matrix also helps identify manipulator configurations where control becomes difficult. These are called singularities, and they occur when the Jacobian Matrix loses its ability to translate between joint and end-effector velocities. By analyzing the Jacobian Matrix, we can avoid these configurations and ensure smooth, controlled motion.

</div>

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<a name="Differential Equations"> </a>
# IV. Differential Equations
<div align="justify">

Imagine a robotic arm in mid-motion. Its joints are at specific positions (angles) and moving at certain velocities. But how does this current state - a combination of positions and velocities - determine the arm's future movement? This is where differential equations come into play.
 These equations act like a set of instructions that describe the dynamic relationship between the manipulator's state (joint positions and velocities) and how those states change over time. They're like a recipe for predicting the robot's future "dance." By solving these equations, we can essentially simulate the arm's motion, understanding how its positions and velocities will evolve over time.

 This simulation capability is invaluable for several reasons. First, it allows us to predict the manipulator's trajectory, ensuring it reaches its target destination precisely. Second, it helps us identify potential problems like collisions or reaching limitations in joint movement. Finally, by simulating different scenarios beforehand, we can refine control strategies and optimize the arm's performance for various tasks.
    
    

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<a name="Singularities"> </a>
# V. Singularities
<div align="justify">
    
Imagine a robotic arm frozen mid-motion, unable to respond to certain joint commands. This perplexing phenomenon is called a singularity. It occurs when the arm adopts a specific configuration that disrupts the relationship between joint movements and the end-effector's behavior. Here's where the magic happens (or rather, doesn't): the Jacobian matrix, the translator between joint velocities and end-effector velocities, loses its effectiveness. In simpler terms, the matrix can no longer accurately predict how the end-effector will respond to certain joint motions. This "missing motion" can manifest in several ways:

-Limited Maneuverability: The arm might struggle to move in specific directions, even though individual joints are functioning correctly.
-Erratic Control: Joint commands might produce unexpected or jerky movements at the end-effector.
-Potential Damage: Operating in a singularity can put excessive stress on the arm's motors and joints, leading to wear and tear.

By understanding singularities and how they affect the Jacobian matrix, we can design motion paths that avoid these problematic configurations. This ensures the robotic arm operates smoothly, efficiently, and most importantly, safely.

</div>

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<a name="Path and Trajectory Planning Utilizing GUI"> </a>
# VI. Path and Trajectory Planning Utilizing GUI
<div align="justify">

Path Planning: Think of it as a map for the robot's end-effector, charting a safe and efficient course around obstacles while respecting the robot's workspace boundaries.

Trajectory Planning: This adds the timing element, specifying speeds and accelerations at each point along the path. Just like a well-choreographed dance, it ensures smooth and controlled movements.

Bridging the Gap: GUIs for Intuitive Control
Imagine a user interface where you control the robot with simple sliders. GUIs built with tools like MATLAB or Python allow visualization of the robot's movements in real-time. Users can manipulate joints through sliders, with the path or trajectory dynamically updating based on their input. This user-friendly platform simplifies robot control and fosters better human-robot interaction.

</div>



<p align="right">(<a href="#readme-top">back to top</a>)</p>


# |References|



<p align="right">(<a href="#readme-top">back to top</a>)</p>
