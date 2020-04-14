# ROS package to mark the triangle model on RVIZ.

This package is to mark the triangle model on RVIZ and output its location of centroid point and three end points in 3D space.  

## Dependencies
1. Eigen3

## Content
1. ground_truth_labeler_node

## 1. ground_truth_labeler_node

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
3. Subscribe the topic **iterativemarkers**   

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
