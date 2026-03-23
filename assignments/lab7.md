# Lab 7: Reinforcement Learning

By now, you're familiar with how much work it takes to accumulate enough data to train AI for robotics.  
Simulating robots is one strategy to generate huge amounts data quickly for robots to learn faster.  
In this lab, we'll begin using Isaac Lab inside of Nvidia's Isaac Sim to explore reinforcement learning (RL).  


## Learning Objectives

- Complete the code for a managed environment in Isaac Lab.
- Train your first RL policy in Isaac Lab.
- Monitor training of the RL policy.
- Play the policy for evaluation.


## Given

- [Gymnasium by OpenAI, maintained by Farama](https://gymnasium.farama.org/)  
    Many robot RL simulators are based on the gym framework initially developed by OpenAI.  
    Reading through the documentation and basic examples is a good introduction for learning more about RL.

- [RL Libraries Included in Isaac Lab](https://isaac-sim.github.io/IsaacLab/main/source/overview/reinforcement-learning/rl_frameworks.html)  
    Integrations like this is why we're using Isaac Lab instead of developing RL in Gazebo or the real robot.  
    Isaac Lab is purpose-built for running RL studies in parallel across thousands of simulations.  


## TODO

1. Create an Isaac Lab "external" project for the so101 arm.
    - In the Isaac Lab container and directory:
    ```bash
    ./isaaclab.sh --new
    ```
    - Task type: External
    - Project path: /techin517/isaac_so101
    - Project name: so101
    - Isaac Lab workflow: Manager-based | single-agent
    - RL library: rsl_rl

2. Copy the python file from this lab folder to the <TODO> directory in your new external project.

3. Fill in the TODOs in the python file.

4. Add the environment to <TODO>.

5. Run reinforcement learning with the following command:
    ```bash
    # TODO
    ```

6. During training, open a new terminal and monitor the training process.  
    You can attach a new terminal to the existing container by running:  
    ```bash
    # TODO
    ```  
    Then use [Tensorboard](TODO) to monitor training by running:   
    ```bash
    # TODO
    ```

7. After training, play the policy using the following command:  
    ```bash
    # TODO
    ```


## Deliverables

1. Submit your completed Isaac Lab managed environment code.

2. Submit a screenshot of your Tensorboard after training is complete.

3. Submit a video of the SO101 in Isaac Lab successfully picking the cube.


## Resources

[Isaac Lab Official Documentation](https://isaac-sim.github.io/IsaacLab/main/index.html)

[Isaac Lab Official Examples](https://isaac-sim.github.io/IsaacLab/main/source/overview/environments.html)

[Isaac Lab Office Hours by Nvidia Omniverse on YouTube](https://www.youtube.com/playlist?list=PL3jK4xNnlCVcnMqm4Lnqa5Bok4_iP5NsK)

[Wheeled Lab by researchers at UW!](https://www.youtube.com/watch?v=Y4b-cz2xB1w)

[Comparison of different RL simulation environments by FS Studio on LinkedIn](https://www.linkedin.com/pulse/comparative-analysis-robotic-simulation-environments-fs-studio-cpjic)
