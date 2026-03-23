# Lab 5: Hand-eye Coordination

Now that we know how to accurately move the robot arm, we need methods for understanding the robot's environment to decide where to move to perform real tasks.  
There are large bodies of work for understanding the world with computer vision.  
In this lab, we will learn how to calibrate and coordinate computer vision with robot arms.


## Learning Objectives

- Calibrate arms and cameras to work together.
- Track the position of an aruco cube using computer vision.
- Control robot arms using information from cameras.
- Attempt to pick arbitrary objects using poses from YOLO 3D.


## Given

- [ros2_handeye_calibration](https://github.com/giuschio/ros2_handeye_calibration)

- [yolo_ros](https://github.com/mgonzs13/yolo_ros)  
    We will use the same ros2 yolo package as last quarter to quickly estimate positions of various objects 


## TODO

1. Use the `ros2_handeye_calibration` package to calibrate the arm with the Realsense D435i depth camera.  

2. Complete the `aruco_cube_position` node in `soa_functions`

3. Complete the `pick_by_position` app in `soa_apps`

4. Use the `pick_by_position` app to pick the cube, using the position `aruco_cube_position`.  
    Record a video demo.  

5. Use the `pick_by_position` app to pick a fake piece of fruit, using the position from the yolo3D topic.  
    Record a video demo.  


## Deliverables

1. Link to your video demo picking the aruco cube.  

2. Link to your video demo picking the fruit.

3. Write a paragraph discussing classic motion planning versus learning-based control.  
    **Do not use AI.**  
    How do the different systems handle manipulating arbitrary objects?  
    What are the data requirements for manipulating with classic versus learned control?  
    How do classic and learned control compare for non-manipulation end-effectors (e.g. milling, drilling, sensing, spraying, sanding, etc.)?  


## FAQ


## Resources

- [Nvidia's Foundation Pose](https://nvlabs.github.io/FoundationPose/)  
    This system uses a 3D representation of objects to infer the pose of objects.  
    As demoed, these poses can be used to accurately manipulate objects.  
    There are many systems for 6-dof pose estimation that compete on the [BOP leaderboard](https://bop.felk.cvut.cz/leaderboards/pose-estimation-unseen-bop23/core-datasets/)  