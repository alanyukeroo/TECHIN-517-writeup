# Lab 4: Pose Goals & Obstacle Avoidance

Forward kinematics is useful and reliable when you know the exact joint-states of your goal position.  
It's far more common that you only have a goal pose where you want the robot end-effector to go.  
In this case, we use "inverse kinematic" solvers to find joint positions for a given end-effector pose.  
Most industrial robot arms have at least 6 Degrees-Of-Freedom (dof) which allows them to move to any arbitrary 6-dof pose within its workspace.  
The SO101 arm is only 5-dof, significantly limiting the reachable poses.  


## Learning Objectives

- Control the SO101 using inverse kinematics.
- Include environment constraints in motion planning.


## Given

- [MoveIt Included Planners](https://moveit.ai/documentation/planners/)  
    MoveIt provides integrations with many planners for inverse kinematics.  
    The default planner is the [Open Motion Planning Library](https://ompl.kavrakilab.org/)

- [MoveIt Planning Scene](https://moveit.picknik.ai/main/doc/tutorials/planning_around_objects/planning_around_objects.html)  
    MoveIt includes tools for planning around obstacles.  
    In this lab, we will use the planning scene to control for the table.  
    Your project should expand on these capabilities to control for other obstacles in your scene.  


## TODO

1. Complete the `move_to_pose_server` in `soa_functions`.

2. Complete `load_planning_scene` in `soa_functions`.

3. Try to move the robot arm through the table in Rviz when using your planning scene.  
    Launch MoveIt, start your planning scene in a second terminal.  
    Use the planning marker in Rviz to bring the robot arm through the table.  
    The robot arm should turn red showing that it isn't possible.  

3. Create a new app called `go_to_poses` in `soa_apps.  `


## Deliverables

1. Submit your code for `move_to_pose_server`.

2. Submit your code for `load_planning_scene`

3. Submit a screenshot of the robot arm turning red when intersecting with the planning scene in Rviz.

4. Submit a link to a video of the robot arm going through poses using your `go_to_poses` app.

5. Write a paragraph discussing forward versus inverse kinematics.  
    **Do not use AI.**  
    In what circumstances would you use each type?  


## FAQ

## Resources

- [Pymoveit2 Example Scripts](https://github.com/AndrejOrsula/pymoveit2/tree/main/examples)

- [MoveIt Obstacle Avoidance Tutorial by Automated Addison](https://www.youtube.com/watch?v=NzknK_SRDjk)