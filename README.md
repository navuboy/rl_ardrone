# rl4uav - Reinforcement Learning for Autonomous navigation of UAV's (indoors) 
## Q-Learning.py
Autonomous Navigation of UAV using Q-Learning (Reinforcement Learning). 
- State: Discrete
- Action: Discrete
- Action space: 5x5 grid space.

Indoor Path Planning and Navigation of an Unmanned Aerial Vehicle (UAV) based on PID+Q-Learning algorithm (Reinforcement Learning). The quadrotor maneuvers towards the goal point, along the uniform grid distribution in the simulation environment(discrete action space) based on the specified reward policy, backed by the simple position based PID controller.

This project was developed at the <b>Advanced Flight Simulation(AFS) Labarotory, IISc, Bangalore</b>.
<p align= "center">
<img src="drone_qlearning.gif/">
</p>

### Prerequisites:
- <a href="http://releases.ubuntu.com/16.04/">Ubuntu 16.04</a> 
- <a href="http://wiki.ros.org/kinetic">ROS Kinetic</a>
- <a href="http://gazebosim.org/">Gazebo 7</a>
- <a href="https://github.com/AutonomyLab/ardrone_autonomy">ArDrone Autonomy ROS Package</a>
- <a href="https://www.tensorflow.org/install/">TensorFLow 1.1.0 (with GPU)</a>
- Python: 2.7

### Reference:  
- Pham, Huy X., et al. <b><a href="https://arxiv.org/abs/1801.05086">Autonomous uav navigation using reinforcement learning.</a></b> arXiv preprint arXiv:1801.05086 (2018).
- Mnih, Volodymyr, et al. <a href="https://storage.googleapis.com/deepmind-media/dqn/DQNNaturePaper.pdf"><b>Human-level control through deep reinforcement learning.</b></a> Nature 518.7540 (2015)

**Project Video: https://goo.gl/zKNQdW**

## DDPG.py 
Deep Deterministic Policy Gradient algorithm is used for autonomous navigation of UAV from start to goal position. This is applicable for continuous action-space domain. (Work in Progress)

### Collaborator(s):
**<a href="https://github.com/ioarun">Arun Kumar</a>** 
