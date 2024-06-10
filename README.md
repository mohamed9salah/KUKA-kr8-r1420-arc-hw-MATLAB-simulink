# KUKA-kr8-r1420-arc-hw-MATLAB-simulink
# Simulation of KUKA robot in matlab simulink and App designer
This project contains a general tab designed by APPDESIGNER. By inserting the parameters of DH table with determining the situation of each link if it is prizmatic or revolute, we find the transformation matrix, Jacobian matrix, joints angles and a 3d plot for visualization of the robot according to inserted DH table. 
The other tab is for KUKA kr8 r1420 arc hw robot in addition to calculating inverse kinematics with Forward and Back Reaching Inverse Kinematics and Cyclic Coordinate Descent methods. 
Using cad files of the robot to create simscape file. 
Creating simulation environment for 6DOF robotic arm using Peter Corke Robotics Toolbox. 
# KUKA KR8-R 1420HW 
- Main assemblies of the manipulator
![image](https://github.com/mohamed9salah/KUKA-kr8-r1420-arc-hw-MATLAB-simulink/assets/138705468/6b24da97-9387-410d-8c1c-2c936d2dc5d3)
1-	Link arm 
2-	Hollow-shaft wrist/arm
3-	Electrical installation
4-	Base frame
5-	Rotating column
# Technical data
![image](https://github.com/mohamed9salah/KUKA-kr8-r1420-arc-hw-MATLAB-simulink/assets/138705468/80decf33-e51d-4a57-a711-55cc1ce221eb)

# axis data KR 8 R1420 HW
![image](https://github.com/mohamed9salah/KUKA-kr8-r1420-arc-hw-MATLAB-simulink/assets/138705468/f6638a77-9708-4d72-8c21-a1432c638c45)
![image](https://github.com/mohamed9salah/KUKA-kr8-r1420-arc-hw-MATLAB-simulink/assets/138705468/f31b56cf-0358-4515-915b-51851537210f)

# Denavit-Hartenberg table parameters 
![image](https://github.com/mohamed9salah/KUKA-kr8-r1420-arc-hw-MATLAB-simulink/assets/138705468/8a8ed697-9864-45c6-8fd6-a6ddd4e16726) ![image](https://github.com/mohamed9salah/KUKA-kr8-r1420-arc-hw-MATLAB-simulink/assets/138705468/2f28d2ce-7618-4368-9f75-3b71e44e1e97)

![image](https://github.com/mohamed9salah/KUKA-kr8-r1420-arc-hw-MATLAB-simulink/assets/138705468/cebf731d-ff6a-49aa-a1b1-bc6d0269cc2a)
![image](https://github.com/mohamed9salah/KUKA-kr8-r1420-arc-hw-MATLAB-simulink/assets/138705468/c324d1ad-49d5-4980-8862-30ccd7049d58)
![image](https://github.com/mohamed9salah/KUKA-kr8-r1420-arc-hw-MATLAB-simulink/assets/138705468/7cae5d5c-829d-4e58-be65-ed991343d96e) ![image](https://github.com/mohamed9salah/KUKA-kr8-r1420-arc-hw-MATLAB-simulink/assets/138705468/19eb9633-002b-4025-9d74-3c6b56e3429d)

# MATLAB
Using app designer, calculating transformation matrix with Jacobian matrix and analyzing inverse kinematics in two methods forward and back reaching inverse kinematics method and cyclic coordinate descent method. Creating a plot visualization for the robot. 
The dh table parameters can be entered by the user. Values used in current calculations to achieve our process: a= [2 0 6 2 0 0], α= [0 -90 0 -90 -90 90], d= [4.5 0 6 0 0 3] and θ= [60 90 0 150 120 0]. Determining the status of each link, Revolute or Prizmatic First tab “general” :

![image](https://github.com/mohamed9salah/KUKA-kr8-r1420-arc-hw-MATLAB-simulink/assets/138705468/a47ce59e-502a-4139-a74e-08ac9be6a83d)

After getting dh table parameters and specifying if the link is revolute R, prismatic P or None, it is planned to calculate T (transformation) matrix and it will be shown a message that parameters are saved to the workspace. 

![image](https://github.com/mohamed9salah/KUKA-kr8-r1420-arc-hw-MATLAB-simulink/assets/138705468/42c86317-d358-4850-a598-30accc687097)

By clicking button of calculate JacobianM and button of plot, it is planned to calculate Jacobian matrix and printing link positions with plotting robot visualization.

![image](https://github.com/mohamed9salah/KUKA-kr8-r1420-arc-hw-MATLAB-simulink/assets/138705468/31f57cec-ac1f-4cf7-8aed-a80bb7047567)

- Determining x y z positions and RPY angles
![image](https://github.com/mohamed9salah/KUKA-kr8-r1420-arc-hw-MATLAB-simulink/assets/138705468/c00448bd-35b0-4429-a169-b7d352b10407)

- for the second tab
  By clicking on load button, it is planned to load the entered dh parameters and link positions with KUKA KR8 R HW visualization. We use the values of alpha, a and d from DH table of KUKA KR8 RHW robot.
  ![image](https://github.com/mohamed9salah/KUKA-kr8-r1420-arc-hw-MATLAB-simulink/assets/138705468/450339d2-959a-4fef-9310-3238de130f3f)

- simscape
  ![image](https://github.com/mohamed9salah/KUKA-kr8-r1420-arc-hw-MATLAB-simulink/assets/138705468/ab33291f-c7d6-43b4-ab08-42252e38a4c2)

# inverse kinematics two methods, Forward And Back Reaching Inverse Kinematics) and CCD (Cyclic Coordinate Descent). 
![image](https://github.com/mohamed9salah/KUKA-kr8-r1420-arc-hw-MATLAB-simulink/assets/138705468/217eb25f-170f-4f53-b7cf-b58ec4a31559) 
![image](https://github.com/mohamed9salah/KUKA-kr8-r1420-arc-hw-MATLAB-simulink/assets/138705468/42487f34-839e-45a2-8a5b-9e52cc281760) ![image](https://github.com/mohamed9salah/KUKA-kr8-r1420-arc-hw-MATLAB-simulink/assets/138705468/d89b96c5-11ce-4c87-8c79-458b8b86c770)





















