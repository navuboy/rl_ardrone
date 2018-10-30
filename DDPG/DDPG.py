#Implementation of Deep Deterministic Policy Gradient for autonomous navigation of ArDrone
#for continuos action space domain

import numpy as np
import tensorflow as tf
import gym
import random
import copy
import math

import roslib
import rospy
import random
import time
import math
from std_srvs.srv import Empty as empty
from std_msgs.msg import Empty
from gazebo_msgs.srv import SetModelConfiguration

from control_msgs.msg import JointControllerState
from sensor_msgs.msg import JointState
from gazebo_msgs.msg import LinkStates
from std_msgs.msg import Float64
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from ardrone_autonomy.msg import Navdata

from ddpg import *

rospy.init_node('control_script')
rate = rospy.Rate(120)

EPISODES = 100000
TEST = 10

pub_takeoff = rospy.Publisher('/ardrone/takeoff', Empty, queue_size=10)
pub_land = rospy.Publisher('/ardrone/land', Empty, queue_size=10)
pub_action = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

reset_simulation = rospy.ServiceProxy('/gazebo/reset_world', empty)
unpause = rospy.ServiceProxy('/gazebo/unpause_physics', empty)
pause = rospy.ServiceProxy('/gazebo/pause_physics', empty)

class DroneState(object):
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.rot_x = 0.0
        self.rot_y = 0.0
        self.rot_z = 0.0
        self.x_dot = 0.0
        self.y_dot = 0.0
        self.z_dot = 0.0
        self.drone_state = [self.x, self.y, self.z, self.x_dot, self.y_dot, self.z_dot]
        self.done = False
        self.is_reset = False

    def set_drone_state(self, state):
        self.drone_state = [state[0], state[1], state[3], state[4]]

    def get_drone_state(self):
        return self.drone_state

    def reset(self):
        rospy.wait_for_service('gazebo/reset_world')
        try:
            reset_simulation()
        except(rospy.ServiceException) as e:
            print "reset_world failed!"

        # rospy.wait_for_service('/gazebo/pause_physics')
        # try:
        #     pause()
        # except (rospy.ServiceException) as e:
        #     print "rospause failed!"

        return env.get_drone_state()

    def step(self, action):
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            unpause()
        except (rospy.ServiceException) as e:
            print "/gazebo/pause_physics service call failed"

        command = Twist()
   
        command.linear.x = action[0]
        command.linear.y = action[1]
        command.linear.z = 0.0
        command.angular.x = action[2]
        command.angular.y = action[3]
        command.angular.z = 0.0


        pub_action.publish(command)

        rate.sleep()

        next_state = env.get_drone_state()
        reward = get_reward(next_state)
        
        if (next_state[0] < -0.1 or next_state[0] > 5.1 or next_state[1] < -0.1 or next_state[1] > 5.1 or reward == 1):
            env.done = True

        return next_state, reward, env.done

env = DroneState()

def get_reward(state):
    reward = 0.0
    if abs(state[0] - 5.0) <= 0.05 and abs(state[1] - 5.0) <= 0.05:
        # goal reached 
        reward = 1.0
    elif (state[0] < -0.1 or state[0] > 5.1 or state[1] < -0.1 or state[1] > 5.1 or reward == 1):
    	reward = -100.0
    else:
        reward = - math.sqrt((5.0-state[0])**2 + (5.0-state[1])**2)
    # if (state[0] < -0.05 or state[0] > 5.05 or state[1] < -0.05 or state[1] > 5.05):
    return reward
#---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------#
def takeoff():
    pub_takeoff.publish(Empty())
#---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------#
def land():
    pub_land.publish(Empty())
#---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------#
def cb_drone_state(data):
    env.set_drone_state([data.pose.pose.position.x,data.pose.pose.position.y, data.pose.pose.position.z, data.twist.twist.linear.x, data.twist.twist.linear.y, data.twist.twist.linear.z])
#---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------#
def cb_drone_navdata(data):
    env.rot_x = data.rotX
    env.rot_y = data.rotY
    env.rot_z = data.rotZ
#---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------#
def subscriber():
    rospy.Subscriber("/ground_truth/state", Odometry, cb_drone_state)
    rospy.Subscriber("/ardrone/navdata", Navdata, cb_drone_navdata)

subscriber()
#---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------#
def main():
    ddpg_env = [4, 4]
    agent = DDPG(ddpg_env)

    for episode in range(EPISODES):
        state = env.reset()
        # Train
        for step in range(16000):
            action = agent.noise_action(state)
            next_state,reward,done = env.step(action)
            agent.perceive(state,action,reward,next_state,done)
            state = next_state
            if done:
                env.done = False
                break
        print "episode:",episode
        # Testing:
        if episode % 100 == 0 and episode > 100:
            print "testing"
            total_reward = 0
            for i in range(TEST):
                state = env.reset()
                for j in range(16000):
                    action = agent.action(state) # direct action for test
                    state,reward,done = env.step(action)
                    total_reward += reward
                    if done:
                        env.done = False
                        break
            ave_reward = total_reward/TEST
            print 'episode: ',episode,'Evaluation Average Reward:',ave_reward

if __name__ == '__main__':
    main()







