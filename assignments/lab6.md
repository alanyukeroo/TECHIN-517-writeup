# Lab 6: The Green Pill: Isaac Sim

This week we will begin using Nvidia's Isaac Sim.  
Isaac Sim is another simulation platform that includes features for [simulating robots, computer-vision synthetic data generation, maintaining digital-twins, reinforcement learning, more](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/index.html).  


## Learning Objectives

- Import a URDF into Isaac Sim.
- Specify parameters required to accurately simulate robot arms.
- Connect information from ROS to Isaac Sim. 


## Given

- [Isaac Sim](https://developer.nvidia.com/isaac/sim)

- [Lychee AI Tutorial for Importing the SO101 into Isaac Sim](https://www.youtube.com/watch?v=AMfEtZ4hyLY)


## TODO

1. Install Isaac Sim and Isaac Lab.  
There are many ways to install Isaac Sim, we're going to use Nvidia's Isaac Lab Docker container.  
    - Clone Nvidia's Isaac Lab repo **into your home directory on the host computer**:
    ```bash
    git clone https://github.com/isaac-sim/IsaacLab.git
    ```
    - Open the repo in VS Code
    - In the file `IsaacLab/docker/.env.base`, change the ISAACSIM_VERSION to 5.0.0
    - Add your class repo as a volume in `IsaacLab/docker/docker-compose.yaml`
    ```yaml
    - existing volumes...
    - type: bind
        source: .isaac-lab-docker-history
        target: ${DOCKER_USER_HOME}/.bash_history
    # TODO: add class repo as a shared volume
    - type: bind
        source: <relative/path/to/your/class/repo>
        target: /techin517
    ```
    - Build the container with ROS2 support
    ```bash
    ./docker/container.py start ros2
    ```
    - Enter the container
    ```bash
    ./docker/container.py enter ros2
    ```
    - Start Isaac Sim
    ```bash
    runapp
    ```
    - Enter `exit` to leave the container, afterwards shut it down with:
    ```bash
    ./docker/container.py stop ros2
    ```

2. Import the SO101 arm into Isaac Sim.
    - In Isaac Sim, go to File > Import and choose  
    `/techin517/so101_ws/src/so101_ros2/so101_description/urdf/so101_newcalib.urdf`
    - Enter the following settings to approximate the physics of the arm in Isaac Sim:
        - Model: Referenced Model
        - USD Output: `/techin517/`
        - Links: Static Base
        - Default Density: 0.0
        - Joints & Drivers: Check "Ignore Mimic"
        - Joint Configuration: Natural Frequency
        - Drive Type: Force
        - Joint settings
            | Name | Target | Frequency | Damping | 
            | - | - | - | - |
            | elbow_flex | position | 10.0 | 0.7 |
            | gripper | position | 10.0 | 0.7 |
            | shoulder_lift | position | 10.0 | 0.7 |
            | shoulder_pan | position | 10.0 | 0.7 |
            | wrist_flex | position | 10.0 | 0.7 |
            | wrist_roll | position | 10.0 | 0.7 |

            Note: these values might change as you enter other values, make sure they're all correct before continuing
        - Colliders: Do not check "Collision From Visuals"
        - Collider Type: Convex Hull
        - Check "Allow Self-Collision"
        - Do not check: "Replace Cyllinders with Capsules"

        All credit to [Lychee AI](https://lycheeai-hub.com/project-so-arm101-x-isaac-sim-x-isaac-lab-tutorial-series/so-arm-import-urdf-to-isaac-sim) for providing these values.

3. Connect Isaac Sim to ROS
    - Go to Tools > Robotics > ROS 2 OmniGraphs > Joint States
    - Articulation Root > World (defaultPrim) > so101_new_calib > base_link
    - Check the box for Publisher
    - Change publisher topic to /isaac_joint_states
    - Check the box for Subscriber
    - Change the subscriber topic to /isaac_joint_command

4. Teleoperate the arm from ROS to Isaac Sim.
    - Press play in Isaac Sim
    - Start teleoping the arm using ROS:
    ```bash
    ros2 launch so101_bringup so101_teleoperate.launch.py mode:=real expert:=human display:=true
    ```
    - Run the following in ROS to connect the topics:
    ```bash
    ros2 run topic_tools relay /leader/joint_states /isaac_joint_command
    ```


## Deliverables

1. Submit a video of yourself teleoperating the arm in Isaac Sim.  
    Include your real-life setup with the follower arm and a view of the simulation screen.  

2. Write a paragraph explaining how you would begin to simulate a brand-new robot?  
    What information would you need?  
    What would you have to develop before approaching Isaac Sim?


## FAQ

**Q:** When I run `runapp` there's a lot of error messages about not having a GPU even though I have Nvidia container toolkit, how do I fix it?  
**A:** Try running `./docker/container.py stop ros2` then restart the container.  


## Resources

[Official Isaac Sim 5.0.0 documentation on importing URDFs](https://docs.isaacsim.omniverse.nvidia.com/5.0.0/importer_exporter/ext_isaacsim_asset_importer_urdf.html)  

[Lychee AI tutorial on importing URDFs into Isaac Sim](https://www.youtube.com/watch?v=KCHmYvYF_6c)