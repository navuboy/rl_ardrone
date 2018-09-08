# RL4UAV - Reinforcement Learning for Autonomous navigation of UAV's (indoors) 
(1) *****Q-Learning.py***** : Autonomous Navigation of UAV using Q-Learning (Reinforcement Learning). 
- State: Discrete
- Action: Discrete
- Action space: 5x5 grid space.

Indoor Path Planning and Navigation of an Unmanned Aerial Vehicle (UAV) based on PID+Q-Learning algorithm (Reinforcement Learning). The quadrotor maneuvers towards the goal point, along the uniform grid distribution in the simulation environment(discrete action space) based on the specified reward policy, backed by the simple position based PID controller.
<p align= "center">
<img src="drone_qlearning.gif/">
</p>

Platforms and Packages: 
- Ubuntu 16.04 
- ROS Kinetic (http://wiki.ros.org/kinetic)
- Gazebo 7 (http://gazebosim.org/)
- TUM ArDrone Autonomy
- TensorFLow 1.1.0 (with GPU)
- Python: 2.7

**Reference:** Pham, H.X., La, H.M., Feil-Seifer, D. and Nguyen, L.V., 2018. Autonomous UAV Navigation Using Reinforcement Learning. arXiv preprint arXiv:1801.05086.
**Project Video: https://goo.gl/zKNQdW**

(2) *****DDPG.py***** : Deep Deterministic Policy Gradient algorithm is used for autonomous navigation of UAV from start to goal position. This is applicable for continuous action-space domain. 
