# ROS package to mark the triangle model on RVIZ.

This package is to mark the ground truth (cube, triangle) on RVIZ.  

## Requirements
1. Eigen3
2. tkinter

## Content
1. ground_truth_labeler (ground_truth_labeler.py)  
2. ground_truth_labeler_node (ground_truth_labeler.cpp)  


## 1. ground_truth_labeler
This is a tool to mark ground truth (cube) by using interactive makrer in UI(implemented by tkinter)  

### (a) Getting Started.
1. put package **ground_truth_labeler** in your workspace and catkin_make the package.  

2. Run the module  
```
roscore
python ~/ground_truth_labeler/src/ground_truth_labeler.py
```

### (b) Visualize on RVIZ
1. Open the RVIZ  
2. Set the fixed frame as your input frame, the default frame is **map**  
3. Subscribe the topic **simple_markers** and **cube_poses**     

You won't see anything for now, you need to create the marker on UI.  

### (c) Create the first model
1. Click **New Marker** button on the User Interface.
2. Enter the numbers and their size you need.
3. Now check the RVIZ.  

The result may seen like this:  
<img src="https://github.com/tom13133/ground_truth_labeler/blob/master/images/labeler.png" width="1000">

* Notice that **tx**, **ty**, **tz** on **Update Marker interface** is not the position of the cube, it is the position of **the control object** (above the cube).

## 2. ground_truth_labeler_node
This is a tool to mark ground truth (triangle model) by using interactive makrer in command line

### (a) Getting Started.
1. put package **ground_truth_labeler** in your workspace and catkin_make the package.  

2. Run the node  
```
roscore
rosrun ground_truth_labeler ground_truth_labeler_node
```

### (b) Visualize on RVIZ
1. Open the RVIZ  
2. Set fixed frame as **map**  
3. Subscribe the topic **simple_markers**  

You won't see anything for now, you need to create the marker by orders.  

### (c) Create the first model
1. Click **2D Nav Goal** button on the upper part of RVIZ interface. (Or press **g**)  
2. Try to drag the green arrow on the RVIZ 3D space.  
3. Now check the terminal you run the **ground_truth_labeler_node**, following the orders to create the model.  

For instance, you can create one model by giving such orders:  
<img src="https://github.com/tom13133/ground_truth_labeler/blob/master/images/orders.png" width="900">

The movable model shown on the RVIZ:  
<img src="https://github.com/tom13133/ground_truth_labeler/blob/master/images/triangle_model.png" width="500">

4. Want to give more orders, back to step 1.

