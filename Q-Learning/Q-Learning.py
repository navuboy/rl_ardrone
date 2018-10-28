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


rospy.init_node('control_script')
rate = rospy.Rate(120)

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
        self.Q = dict()
        self.epsilon = 1.0
        self.epsilon_max = 1.0
        self.epsilon_min = 0.01
        self.alpha = 0.8
        self.gamma = 0.9

    def build_state(self, state):

        state = str(int(round(state[0])))+'_'+str(int(round(state[1])))
            
        return state

    def get_maxQ(self, state):
        maxQ = -10000000
        for action in self.Q[state]:
            if self.Q[state][action] > maxQ:
                maxQ = self.Q[state][action]

        return maxQ 

    def createQ(self, state):
        if state not in self.Q.keys():
            self.Q[state] = self.Q.get(state, {'0':0.0, '1':0.0, '2':0.0, '3':0.0})

        return

    def choose_action(self, state):
        valid_actions = ['0', '1', '2', '3']
        if random.random() < self.epsilon:
            action = random.choice(valid_actions)
            print "random: ", action
        else:
            actions = []
            maxQ = self.get_maxQ(state)
            for action in self.Q[state]:
                if self.Q[state][action] == float(maxQ):
                    actions.append(action)
            action = random.choice(actions)
            print "greedy: ", action

        return action

    def learn(self, state, action, reward, next_state):
        maxQ_next_state = env.get_maxQ(next_state)
        # self.Q[state][action] = self.Q[state][action] + self.alpha*(reward - self.Q[state][action])
        self.Q[state][action] = (1 - self.alpha)*self.Q[state][action] + self.alpha*(self.gamma*(reward + maxQ_next_state))

        return


    def set_drone_state(self, state):
        self.drone_state = [state[0], state[1], state[2], state[3], state[4], state[5]]

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

        print "called reset()"
        return env.get_drone_state()

    def step(self, action):
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            unpause()
        except (rospy.ServiceException) as e:
            print "/gazebo/pause_physics service call failed"

        command = Twist()
        
        max_vel = 1.0
        
        current_state = env.get_drone_state()
        current_state_x = int(round(current_state[0]))
        current_state_y = int(round(current_state[1]))
        current_state_z = int(current_state[2])
        current_state_rot_z = int(env.rot_z)
        goal_state_rot_z = 0.0
        goal_state_x = 0
        goal_state_y = current_state_y
        goal_state_z = 1.0

        pid.last_error_x = 0.0
        pid.last_error_y = 0.0
        pid.last_error_z = 0.0
        pid.last_error_rot_z = 0.0

        if action == 0:
            goal_state_x = current_state_x + 1
            goal_state_y = current_state_y
            while True:

                u_x, u_y, u_z, u_rot_z = pid.run(current_state_x, goal_state_x, current_state_y, goal_state_y, current_state_z, goal_state_z, current_state_rot_z, goal_state_rot_z)                
                command.linear.x = u_x
                command.linear.y = u_y
                command.linear.z = u_z
                command.angular.x = 0.0
                command.angular.y = 0.0
                command.angular.z = u_rot_z
                pub_action.publish(command)
                if abs(goal_state_x - current_state_x ) <= 0.05 and abs(goal_state_y - current_state_y) <= 0.05 \
                and abs(goal_state_z - current_state_z) <= 0.05 and abs(goal_state_rot_z - current_state_rot_z ) <= 0.05:
                    break
                current_state = env.get_drone_state()
                current_state_x = (current_state[0])
                current_state_y = (current_state[1])
                current_state_z = current_state[2]
                current_state_rot_z = env.rot_z
                rate.sleep()

        if action == 1:
            goal_state_x = current_state_x - 1
            goal_state_y = current_state_y
            while True:
                u_x, u_y, u_z, u_rot_z = pid.run(current_state_x, goal_state_x, current_state_y, goal_state_y, current_state_z, goal_state_z, current_state_rot_z, goal_state_rot_z)
                command.linear.x = u_x
                command.linear.y = u_y
                command.linear.z = u_z
                command.angular.x = 0.0
                command.angular.y = 0.0
                command.angular.z = u_rot_z
                pub_action.publish(command)
                if abs(goal_state_x - current_state_x ) <= 0.05 and abs(goal_state_y - current_state_y) <= 0.05 \
                and abs(goal_state_z - current_state_z) <= 0.05 and abs(goal_state_rot_z - current_state_rot_z ) <= 0.05:
                    break
                current_state = env.get_drone_state()
                current_state_x = (current_state[0])
                current_state_y = (current_state[1])
                current_state_z = current_state[2]
                current_state_rot_z = env.rot_z
                rate.sleep()

        if action == 2: 
            goal_state_y = current_state_y + 1
            goal_state_x = current_state_x
            while True:
                u_x, u_y, u_z, u_rot_z = pid.run(current_state_x, goal_state_x, current_state_y, goal_state_y, current_state_z, goal_state_z, current_state_rot_z, goal_state_rot_z)
                command.linear.x = u_x
                command.linear.y = u_y
                command.linear.z = u_z
                command.angular.x = 0.0
                command.angular.y = 0.0
                command.angular.z = u_rot_z
                pub_action.publish(command)
                if abs(goal_state_y - current_state_y ) <= 0.05 and abs(goal_state_x - current_state_x ) <= 0.05 \
                and abs(goal_state_z - current_state_z) <= 0.05 and abs(goal_state_rot_z - current_state_rot_z ) <= 0.05:   
                    break
                current_state = env.get_drone_state()
                current_state_x = (current_state[0])
                current_state_y = (current_state[1])
                current_state_z = current_state[2]
                current_state_rot_z = env.rot_z
                rate.sleep()

        if action == 3: 
            goal_state = current_state_y - 1
            goal_state_x = current_state_x
            while True:
                u_x, u_y, u_z, u_rot_z = pid.run(current_state_x, goal_state_x, current_state_y, goal_state_y, current_state_z, goal_state_z, current_state_rot_z, goal_state_rot_z)
                command.linear.x = u_x
                command.linear.y = u_y
                command.linear.z = u_z
                command.angular.x = 0.0
                command.angular.y = 0.0
                command.angular.z = u_rot_z
                pub_action.publish(command)
                if abs(goal_state_y - current_state_y ) <= 0.05 and abs(goal_state_x - current_state_x ) <= 0.05 \
                and abs(goal_state_z - current_state_z) <= 0.05 and abs(goal_state_rot_z - current_state_rot_z ) <= 0.05:   
                    break
                current_state = env.get_drone_state()
                current_state_x = (current_state[0])
                current_state_y = (current_state[1])
                current_state_z = current_state[2]
                current_state_rot_z = env.rot_z
                rate.sleep()

        next_state = env.get_drone_state()
        reward = get_reward(next_state)
        
        if (next_state[0] < -0.05 or next_state[0] > 5.05 or next_state[1] < -0.05 or next_state[1] > 5.05 or reward == 100):
            env.done = True

        return next_state, reward, env.done

class PIDController(object):
    def __init__(self, kp, kd=0, ki=0):
        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.last_error_x = 0.0
        self.last_error_y = 0.0
        self.last_error_z = 0.0
        self.last_error_rot_z = 0.0
        self.last_iterm_x = 0.0
        self.last_iterm_y = 0.0
        self.last_iterm_z = 0.0
        self.last_iterm_rot_z = 0.0
        self.time_delta = 0.00833

    def run(self, current_x, goal_x, current_y, goal_y, current_z, goal_z, current_rot_z, goal_rot_z):
        error_x = goal_x - current_x
        self.last_iterm_x += self.ki*self.last_iterm_x*self.time_delta
        error_x_delta = error_x - self.last_error_x
        u_x = self.kp*error_x + (self.kd*error_x_delta)*self.time_delta + self.last_iterm_x
        self.last_error_x = error_x

        error_y = goal_y - current_y
        self.last_iterm_y += self.ki*self.last_iterm_y*self.time_delta
        error_y_delta = error_y - self.last_error_y
        u_y = self.kp*error_y + (self.kd*error_y_delta)*self.time_delta + self.last_iterm_y
        self.last_error_y = error_y

        error_z = goal_z - current_z
        self.last_iterm_z += self.ki*self.last_iterm_z*self.time_delta
        error_z_delta = error_z - self.last_error_z
        u_z = self.kp*error_z + (self.kd*error_z_delta)*self.time_delta + self.last_iterm_z
        self.last_error_z = error_z

        error_rot_z = goal_rot_z - current_rot_z
        self.last_iterm_rot_z += self.ki*self.last_iterm_rot_z*self.time_delta
        error_rot_z_delta = error_rot_z - self.last_error_rot_z
        u_rot_z = self.kp*error_rot_z + (self.kd*error_rot_z_delta)*self.time_delta  + self.last_iterm_rot_z
        self.last_error_rot_z = error_rot_z
        
        return u_x, u_y, u_z, u_rot_z

env = DroneState()
pid = PIDController(0.5, 0.00416, 0.001)

def get_reward(state):
    reward = 0.0
    if abs(state[0] - 5.0) <= 0.05 and abs(state[1] - 5.0) <= 0.05:
        # goal reached 
        reward = 100
    else:
        reward = -1
    # if (state[0] < -0.05 or state[0] > 5.05 or state[1] < -0.05 or state[1] > 5.05):
    return reward

def takeoff():
    pub_takeoff.publish(Empty())

def land():
    pub_land.publish(Empty())

def cb_drone_state(data):
    env.set_drone_state([data.pose.pose.position.x,data.pose.pose.position.y, data.pose.pose.position.z, data.twist.twist.linear.x, data.twist.twist.linear.y, data.twist.twist.linear.z])

def cb_drone_navdata(data):
    env.rot_x = data.rotX
    env.rot_y = data.rotY
    env.rot_z = data.rotZ

def subscriber():
    rospy.Subscriber("/ground_truth/state", Odometry, cb_drone_state)
    rospy.Subscriber("/ardrone/navdata", Navdata, cb_drone_navdata)


num_episodes = 10000

tf.reset_default_graph()

subscriber()

while not rospy.is_shutdown():
    memory_states = []
    memory_targets = []
    for _ in xrange(num_episodes):
        state = env.reset()
        env.done = False
        total_reward = 0
        rospy.sleep(0.25)
        while env.done == False:
            #print i
            state = env.build_state(state)
            env.createQ(state)
            #print state
            #ep_states.append(state)
            #print memory_states
            print "Q Table :", env.Q
            action = env.choose_action(state)
            env.epsilon = env.epsilon_min + (env.epsilon_max - env.epsilon_min)*(math.exp(-0.01*_))
                    
            #take an e-greedy action
            next_state, reward, done = env.step(int(action))
            
            total_reward += reward

            next_state_temp = env.build_state(next_state)
            env.createQ(next_state_temp)
            
            env.learn(state, action, reward, next_state_temp)

            state = next_state

        print "reward in episode ",_," is: ",total_reward






