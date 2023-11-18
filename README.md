# Design, and Implementation of a Perception Stack for Autonomous Driving
## Overview
In this project, you will design, and implement a perception stack for autonomous driving vehicles.
To do this, we will give you a set of 3D point clouds that we have captured from a LiDAR mounted at an intersection.
From these point clouds, at each frame, you are supposed to extract 3D points belonging to vehicles.
From those points, you will estimate, for each vehicle, its position, bounding box, and motion vector.
The bounding box must be a tight fitting bounding box around a vehicle's 3D points.
The position of the vehicle is defined by the center point of the vehicle.
The motion vector is 3D vector that gives the instantaneous velocity of the vehicle.
You can calculate this by subtracting the vehicle's position in the previous frame from the current frame.

## Design Goals
The two design goals for autonomous driving are accuracy, and latency.
Accuracy (self-explanatory) is how accurately you can estimate a vehicle's position, its bounding box, and its motion vector.
We want high accuracy.
Latency represents how long it takes your perception stack to process a single frame.
We want latency to be low.

## Resources
Because you will be working with 3D data, we recommend that you use the Open3D library in your implementation.
Open3D has a Python implementation which should significantly speed up your development.

## Team
You will complete this semester project with two individuals per team.
You are free to choose any team member from within the class.
There are 40 students in the class, so we should have 20 teams.
If you would like to do the project on your own, you are free to do so too.
If you need help finding a team member, please speak with the TA.
We will randomly assign you a team member.

## Detailed Instructions
There are no detailed instructions.
You are free to design your stack, and implement it in any way you like (keeping in view the design goals).
Unlike your assignments in which you had detailed instructions, we want you to explore how to apply some of the techniques you have learnt about in your class.

## Collaborating Vs. Cheating
We encourage you to discuss the design of your perception stack with other teams.
That said, sharing your code with others is not acceptable.
Once all students have submitted their projects, we will run code checking tools on your source code that will help us identify if someone has copied others' code.
If we find that code has been shared, all teams involved will receive a 0/25 on their semester project.

## Grading Rubric
We will use an automated script to grade your projects.
Accuracy will make up 80\% of your grade.
Latency will account for 20\% of it.
For accuracy, at each frame, we will compare the vehicle positions, bounding boxes, and motion vectors that your perception stack estimates with ground truth (actual values).
Of the 80\% for accuracy, vehicle positions make up 40\%, motion vectors 30\%, and bounding boxes 10\%.
In the 20\% for latency, you will receive 10\% if your processing time per frame is less than 0.5 seconds.
You will receive an additional 5\% if you processing time per frame is less than 0.2 seconds.
And an additional 5\% if your processing time per frame is less than 0.1 seconds.
We will measure the processing time on a single machine for all teams.
In addition, we will award 5 extra points (20\% of your grade) to 5 teams with the lowest latency.
For this, you need to have already got 20\% for latency.

## Output Format
Please make a separate directory in your project folder to store your processed results.
Name the directory as ```perception_results/```.
In this directory, you will save a csv for each frame.
The name for each csv should be ```frame_n.csv``` where ```n``` represents the frame number.
Each csv should have the following header:
```
vehicle_id,position_x,position_y,position_z,mvec_x,mvec_y,mvec_z,bbox_x_min,bbox_x_max,bbox_y_min,bbox_y_max,bbox_z_min,bbox_z_max
```
After this header, for each vehicle you detected in that frame, you will write the vehicle position, motion vector, and bounding box.
Our grading script expects this format.
If you do not follow it, you will lose points.

_Important_:
Your code should process one point cloud, then write results to file, and then move to the next one.
You cannot store all results, process them together, and then write to file.

## Visualization
As you process these point clouds, please display your results on a visualization tool as well.
Open3D has a visualizer that you can use.
This visualizer should show only the 3D points belonging to vehicles.
Please color each vehicle's point differently.
In addition, draw a bounding box around each vehicle as well.

# Setting Up the Environment
## Virtual Environment
```
python3 -m venv 431_project_venv
source 431_project_venv/bin/activate
pip install open3d
```

## Dataset
```
unzip Dataset.zip
```

