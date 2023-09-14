import rclpy
from rclpy.node import Node
import rclpy.qos
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Imu
import numpy as np
import time
from pynput import keyboard
import math
import py_trees
import functools
import sys
from py_trees.common import Status
import argparse

class Turtlebot3HighLevelControl(Node):
    def __init__(self):
        super().__init__('turtlebot3_HighLevelControl_node')
        
        self.left = True
        
        self.user_input = ""          # Align Left, Anti-clockwise circuit
        while(self.user_input != "y" and self.user_input != "n"):
            # Ask the user for a number
            self.user_input = input("Do you want the robot to perform anti-clockwise (align left) circuit? [y/n]: ")
            try:
                if(self.user_input == 'y'):
                    print("You entered "+ self.user_input)
                    self.left = True
                elif (self.user_input == 'n'):
                    print("You entered "+ self.user_input)
                    self.left = False
            except ValueError:
                # If the input is not a valid integer, handle the error
                print("Invalid input. Please enter a valid integer.")

        print("INIT")
        # definition of publisher and subscriber object to /cmd_vel and /scan 
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 1)
        self.subscription = self.create_subscription(LaserScan, '/scan', self.laser_callback, rclpy.qos.qos_profile_sensor_data)
        
        # initial state of FSM
        self.state_ = 0

        #initialization dict of lidar regions
        self.regions = {
            'front': 0,
            'right': 0,
            'left': 0,
        }
        # definition of dict with state of FSM
        self.state_dict_ = {
            0: 'find the wall',
            1: 'align left',
            2: 'follow the wall',
            3: 'align right',
            4: 'rotate 180',
            5: 'rewind',
        }
        # velocity command
        self.msg = Twist()

        # distance threshold to the wall
        self.th = 0.30
    
        #timer_period = 0.1  # seconds
        #self.timer = self.create_timer(timer_period, self.control_loop)

        self.ranges = []
        self.ranges_left = []
        self.ranges_right = []
        self.ranges_front_left = []
        self.ranges_front_right = []
        self.max_linear_velocity = 0.20
        self.max_angular_velocity = 2.80   

        self.dist = 0.0

        self.counter = 0
        self.list = []

        self.angular_velocity = []
        self.velocity = []

        self.add_vel = True

        self.current_angle=0
        self.t0 = time.time()
        self.key_pressed = False

        self.angles = []

    def stop_robot(self):
        msg_pub = Twist()
        msg_pub.linear.x  = 0.0
        msg_pub.angular.z = 0.0
        self.publisher_.publish(msg_pub)

    # loop each 0.1 seconds
    def control_loop(self):
        print("\033[32m[Control Loop]\033[0m")
        print("Linear:", self.msg.linear.x)
        #print(self.velocity)
        #print(len(self.velocity))

        #print("Angular:", self.msg.angular.z)
        #print(self.angular_velocity)
        
        if (self.add_vel):
            self.angular_velocity.append(self.msg.angular.z)
            self.velocity.append(self.msg.linear.x)

        self.publisher_.publish(self.msg)
        return Status.SUCCESS


    # laser scanner callback
    def laser_callback(self, msg):
        # populate the lidar reagions with minimum distance 
        # where you find <ranges> you have to read from msg the desired interval
        # I suggesto to do parametric the ranges in this way you don't have to change the value for the real robot 
        
        print("\033[32mLaser callback\033[0m")
        self.ranges = msg.ranges 
        # Togliere i NaN
        # Costruire la lista di angoli 

        # check sanity of ranges array
        if len(msg.ranges) < 100:
            self.linear_vel = 0.0
            self.angular_vel = 0.0
            return 

        self.ranges_left.clear()
        self.ranges_right.clear()
        self.ranges_front_left.clear()
        self.ranges_front_right.clear()
                
        # Check if data is none before saving
        for i in range(0, int(len(self.ranges)/12)):
            dist = np.nan_to_num(self.ranges[i], nan = 2.0)
            #if(np.isnan(self.ranges[i])):
            #    print("NAN!")
            self.ranges_front_right.append(dist)

        #for i in range(45, 135):
        for i in range(int(len(self.ranges)/8), int(len(self.ranges)*3/8)):
            dist = np.nan_to_num(self.ranges[i], nan = 2.0)
            #if(np.isnan(self.ranges[i])):
            #    print("NAN!")
            self.ranges_right.append(dist)

        #for i in range(225, 315):
        for i in range(int(len(self.ranges)*5/8), int(len(self.ranges)*7/8)):
            dist = np.nan_to_num(self.ranges[i], nan = 2.0)
            #if(np.isnan(self.ranges[i])):
            #    print("NAN!")
            self.ranges_left.append(dist)

        #for i in range(330, 360):
        for i in range(int(len(self.ranges)*11/12), len(self.ranges)-1):
            dist = np.nan_to_num(self.ranges[i], nan = 2.0)
            #if(np.isnan(self.ranges[i])):
            #    print("NAN!")
            self.ranges_front_left.append(dist)
      
        self.regions = {
        'front':  min(min(min(self.ranges_front_left), 10), min(min(self.ranges_front_right), 10)),
        'right':  min(min(self.ranges_left), 10),
        'left':  min(min(self.ranges_right), 10)
        }

    def E1_find_wall_condition(self):
        print("RIGHT: ", self.regions['right'])
        print("LEFT: ", self.regions['left'])
        print("FRONT: ", self.regions['front'])
        print("Timestamp: ", self.t0)
        print("Current Angle: ", self.current_angle)
        print("Is key pressed? : ", self.key_pressed)
        
        if(self.regions['right'] > self.th and self.regions['front'] > self.th):
            return Status.SUCCESS
        else:
            return Status.FAILURE
            
    def find_wall_action(self):
        print("\033[32m[Find Wall]\033[0m")
        dist = self.ranges[0]
        self.angular_vel = 0.0
        if dist <= self.th:
            velocity = 0.0
        else:
            velocity = dist*dist
        
        self.dist = dist
        self.msg.linear.x = velocity if velocity < self.max_linear_velocity else self.max_linear_velocity
        self.msg.angular.z = 0.0

        return Status.SUCCESS

    
    def E2_align_left_condition(self):
        if(self.regions['front'] < self.th ):
            return Status.SUCCESS
        else:
            return Status.FAILURE
    
    def align_left_action(self):
        print("\033[32m[Align left]\033[0m")
        # write velocity commands using the class variable self.msg
        # Rotate the robot in place with a defined angular velocity
        self.msg.linear.x = 0.0
        self.msg.angular.z = +0.1
        return Status.SUCCESS
    
    def align_right_action(self):
        print("\033[32m[Align right]\033[0m")
        # write velocity commands using the class variable self.msg
        # Rotate the robot in place with a defined angular velocity
        self.msg.linear.x = 0.0
        self.msg.angular.z = -0.1
        return Status.SUCCESS
        
    def E3_follow_wall_condition(self):
        #if(self.regions['front'] > self.th and self.regions['left'] > self.th and self.regions['right'] < self.th):
        if(self.regions['front'] > self.th and self.regions['right'] < self.th):
            return Status.SUCCESS
        else:
            return Status.FAILURE
        
    def follow_wall_action(self):
        print("\033[32m[Follow wall]\033[0m")
        # write velocity commands using the class variable self.msg
        # Go straight with a defined linear velocity
        self.msg.angular.z = 0.0

        dist = self.ranges[0]
        velocity = dist*dist
        max_linear_velocity = 0.2
        self.msg.linear.x = velocity if velocity < max_linear_velocity else max_linear_velocity
        
        return Status.SUCCESS
    
    def key_flag_cond(self):
        if(self.key_pressed):
            return Status.SUCCESS
        else:
            self.add_vel = True
            return Status.FAILURE

    def rotate_condition(self):
        print("current angle", self.current_angle)
        print("current angular velocity", self.msg.angular.z)
        print("self.time: ", self.t0)

        print(self.velocity)

        if(self.current_angle < math.pi):
            return Status.SUCCESS
        else:
            return Status.FAILURE

    def rotate_action(self):
        print("\033[32m[Rotate]\033[0m")
        self.msg.linear.x = 0.0
        self.msg.angular.z = 0.2 
    
        self.add_vel = False
        print("Is key pressed? : ", self.key_pressed)
        t1 = time.time()
        self.current_angle = self.msg.angular.z * (t1 - self.t0)
        
        return Status.SUCCESS
    
    def not_empty_condition(self):
        if len(self.velocity) > 0:
            return Status.SUCCESS
        else:
            return Status.FAILURE

    def reverse_action(self):
        print("\033[32m[Reverse]\033[0m")
        self.msg.linear.x = self.velocity.pop()
        self.msg.angular.z = - self.angular_velocity.pop()

        if len(self.velocity) == 0:
            self.key_pressed = False
            self.add_vel = True

        return Status.SUCCESS
    
    def safety_modeL(self):
        dist = self.regions['front']
        critical_distance = self.th - 0.05 
        safety_velocity = 0.0

        if dist <= critical_distance:

            print("\033[91m[Safety mode activation]")
            
            if(self.velocity[-1] == 0.0):
                return Status.FAILURE
            
            print("Velocity to be reproduced: ", self.velocity[-1])
            print("Actual velocity performed:", safety_velocity, "\033[0m")
            
            self.msg.linear.x = safety_velocity
            self.msg.angular.z = -0.05
            return Status.SUCCESS
        
        else:
            #safety_velocity = dist*dist
            return Status.FAILURE
    
    def safety_modeR(self):
        dist = self.regions['front']
        critical_distance = self.th - 0.05 
        safety_velocity = 0.0
        
        if dist <= critical_distance:
            print("\033[91m[Safety mode activation]")

            if(self.velocity[-1] == 0.0):
                return Status.FAILURE
            
            print("Velocity to be reproduced: ", self.velocity[-1])
            print("Actual velocity performed:", safety_velocity, "\033[0m")
            
            self.msg.linear.x = safety_velocity
            self.msg.angular.z = 0.05
            return Status.SUCCESS
        
        else:
            #safety_velocity = dist*dist
            return Status.FAILURE


class Root(py_trees.composites.Sequence):
    def __init__(self, robot):
        super().__init__(name="Root")
        self.robot = robot
        
        self.add_child(self.root())
        self.add_child(Control_loop(self.robot))
        
    def root(self):
        selector = py_trees.composites.Selector(name="Rewind / Wall follower")

        selector.add_child(self.selector_rewind1()) # Behaviour of Rewind
        selector.add_child(self.selector()) # Behaviour of Wall Follower

        return selector

    def sequence1(self):
        sequence = py_trees.composites.Sequence(name="Find Wall")
        
        sequence.add_child(E1_find_wall_condition(self.robot))
        sequence.add_child(Find_wall_action(self.robot))
        
        return sequence
    
    def sequence2(self):
        sequence = py_trees.composites.Sequence(name="Align")
        
        sequence.add_child(E2_align_left_condition(self.robot))
        if(self.robot.left):
            sequence.add_child(Align_left_action(self.robot))
        else:
            sequence.add_child(Align_right_action(self.robot))

        return sequence
    
    def sequence3(self):
        sequence = py_trees.composites.Sequence(name="Follow Wall")
        
        sequence.add_child(E3_follow_wall_condition(self.robot))
        sequence.add_child(Follow_wall_action(self.robot))
        
        return sequence

    def selector(self):
        sequence = py_trees.composites.Selector(name="Selector wall follower behaviour")
        
        sequence.add_child(self.sequence1())
        sequence.add_child(self.sequence2())
        sequence.add_child(self.sequence3())
        
        return sequence

    def sequence1_rewind(self):
        sequence = py_trees.composites.Sequence(name="Rotation action")
        
        sequence.add_child(Key_flag_cond(self.robot))
        sequence.add_child(Rotate_condition(self.robot))
        sequence.add_child(Rotate_action(self.robot))

        return sequence
    
    def sequence2_rewind(self):
        sequence = py_trees.composites.Sequence(name="Reverse action")
        
        sequence.add_child(Key_flag_cond(self.robot))
        sequence.add_child(Not_empty_condition(self.robot))
        sequence.add_child(self.selector_reverse())
        
        return sequence

    def selector_reverse(self):
        selector = py_trees.composites.Selector(name="Safe mode / Reverse")
        
        if(self.robot.left):
            selector.add_child(Safety_modeL(self.robot))
        else:
            selector.add_child(Safety_modeR(self.robot))
        #selector.add_child(Safety_mode(self.robot))
        selector.add_child(Reverse_action(self.robot))
        
        return selector
    
    def selector_rewind1(self):
        sequence = py_trees.composites.Selector(name="Rotation / Reverse")
        
        sequence.add_child(self.selector_rewind2())
        sequence.add_child(self.sequence2_rewind())
        
        return sequence

    def selector_rewind2(self):
        sequence = py_trees.composites.Selector(name="Rotation / Key press")
        
        sequence.add_child(self.sequence1_rewind())
        sequence.add_child(KeyPressCondition(self.robot, 'a', timeout=100.0))
        
        return sequence
   

class Control_loop(py_trees.behaviour.Behaviour):
    def __init__(self, robot):
        super().__init__(name="Control Loop")
        self.robot = robot

    def update(self):
        return self.robot.control_loop()

class KeyPressCondition(py_trees.behaviour.Behaviour):
    def __init__(self, robot, key, timeout=2.0):
        """
        Initialize the behavior with the specified key and optional timeout.

        Args:
            key (str): The key to wait for.
            timeout (float): Maximum time (in seconds) to wait for the key press.
        """
        super().__init__(name=f"KeyPressCondition")
        self.robot = robot
        self.key = key
        self.timeout = timeout
        self.start_time = None
        self.listener = keyboard.Listener(on_press=self.on_key_press)
        self.key_pressed = False
        self.listener.start()

    def on_key_press(self, key):
        print("KEYPRESSED")
        
        self.key_pressed = True
        
        self.robot.key_pressed = not(self.robot.key_pressed)

        self.robot.current_angle = 0
        self.robot.t0 = time.time()
        
        print(self.robot.current_angle)
        print(self.robot.t0)

        if not(self.robot.add_vel): 
            self.robot.velocity.clear()
            self.robot.angular_velocity.clear()
            print("\033[91mERASED!\033[0m")

    def initialise(self):
        self.start_time = time.time()
        self.key_pressed = False

    def update(self):
        if (time.time() - self.start_time) >= self.timeout:
            print("TIME-OUT")

        if self.key_pressed or (time.time() - self.start_time) >= self.timeout:
            return Status.SUCCESS if self.key_pressed else Status.FAILURE
        return Status.FAILURE
    

class E1_find_wall_condition(py_trees.behaviour.Behaviour):
    def __init__(self, robot):
        super().__init__(name="E1")
        self.robot = robot

    def update(self):
        return self.robot.E1_find_wall_condition()

class Find_wall_action(py_trees.behaviour.Behaviour):
    def __init__(self, robot):
        super().__init__(name="Find wall")
        self.robot = robot

    def update(self):
        return self.robot.find_wall_action()

class E2_align_left_condition(py_trees.behaviour.Behaviour):
    def __init__(self, robot):
        super().__init__(name="E2")
        self.robot = robot

    def update(self):
        return self.robot.E2_align_left_condition()

class Align_left_action(py_trees.behaviour.Behaviour):
    def __init__(self, robot):
        super().__init__(name="Align left")
        self.robot = robot

    def update(self):
        return self.robot.align_left_action()
    
class Align_right_action(py_trees.behaviour.Behaviour):
    def __init__(self, robot):
        super().__init__(name="Align right")
        self.robot = robot

    def update(self):
        return self.robot.align_right_action()
    
class E3_follow_wall_condition(py_trees.behaviour.Behaviour):
    def __init__(self, robot):
        super().__init__(name="E3")
        self.robot = robot

    def update(self):
        return self.robot.E3_follow_wall_condition()

class Follow_wall_action(py_trees.behaviour.Behaviour):
    def __init__(self, robot):
        super().__init__(name="Follow wall")
        self.robot = robot

    def update(self):
        return self.robot.follow_wall_action()

class Rotate_action(py_trees.behaviour.Behaviour):
    def __init__(self, robot):
        super().__init__(name="Rotate action")
        self.robot = robot
        
    def update(self):
        return self.robot.rotate_action()

class Rotate_condition(py_trees.behaviour.Behaviour):
    def __init__(self, robot):
        super().__init__(name="Rotate condition")
        self.robot = robot

    def update(self):
        return self.robot.rotate_condition()
    
class Key_flag_cond(py_trees.behaviour.Behaviour):
    def __init__(self, robot):
        super().__init__(name="Key flag condition")
        self.robot = robot

    def update(self):
        return self.robot.key_flag_cond()
    
class Not_empty_condition(py_trees.behaviour.Behaviour):
    def __init__(self, robot):
        super().__init__(name="Not empty condition")
        self.robot = robot

    def update(self):
        return self.robot.not_empty_condition()
    
class Reverse_action(py_trees.behaviour.Behaviour):
    def __init__(self, robot):
        super().__init__(name="Reverse")
        self.robot = robot

    def update(self):
        return self.robot.reverse_action()

class Safety_modeL(py_trees.behaviour.Behaviour):
    def __init__(self, robot):
        super().__init__(name="Safety mode L")
        self.robot = robot

    def update(self):
        return self.robot.safety_modeL()
    
class Safety_modeR(py_trees.behaviour.Behaviour):
    def __init__(self, robot):
        super().__init__(name="Safety mode R")
        self.robot = robot

    def update(self):
        return self.robot.safety_modeR()

def post_tick_handler(snapshot_visitor, behaviour_tree):
    
    print(
        py_trees.display.unicode_tree(
            behaviour_tree.root,
            visited=snapshot_visitor.visited,
            previously_visited=snapshot_visitor.visited
        )
    )
    print("-------------------finish tick-------------------------")

    
def main(args=None):
    rclpy.init(args=args)

    parser = argparse.ArgumentParser()
    parser.add_argument("--render", action="store_true", help="Render the behavior tree")
    args = parser.parse_args(args)

    #py_trees.logging.level = py_trees.logging.Level.DEBUG

    my_robot = Turtlebot3HighLevelControl()
    root = Root(my_robot)

    ####################
    # Rendering
    ####################
    # Render the tree
    # py_trees.display.render_dot_tree(root)
    render = args.render 
    if render:
        py_trees.display.render_dot_tree(root)
        sys.exit()

    ####################
    # Execute
    ####################
    tree = py_trees.trees.BehaviourTree(root)
    
    # DEBUG
    snapshot_visitor = py_trees.visitors.SnapshotVisitor()
    tree.add_post_tick_handler(
                        functools.partial(post_tick_handler,
                                            snapshot_visitor))
    tree.visitors.append(snapshot_visitor)
    #####

    tree.setup(timeout=15)

    # Print the ASCII representation of the behavior tree
    #print(py_trees.display.unicode_tree(root))

    print("Helloworld")
    
    i = 0
    while(1):        
        try:
            start_time = time.time()
            i = i + 1
            print("\n--------- Tick {0} ---------\n".format(i))
            rclpy.spin_once(my_robot)

            root.tick_once()
            
            #print("\n")
            print("{}".format(py_trees.display.unicode_tree(root, show_status=True)))
            
            elapsed_time = time.time() - start_time
            print("ELAPSED TIME", elapsed_time)
            
            # Sleep to ensure a minimum tick duration (e.g., 0.1 seconds)
            #if elapsed_time < 0.5:
            #    time.sleep(0.5 - elapsed_time)
            #else:
            #    print("\033[91mFAIL\033[0m")
            #time.sleep(0.1)
        except KeyboardInterrupt:
            print("\033[91mSTOP!\033[0m")
            my_robot.stop_robot()
            time.sleep(0.5)
            break
    print("\nWait for shutdown")

    rclpy.shutdown()

    print("\nFinish")

if __name__ == "__main__":
    main()
