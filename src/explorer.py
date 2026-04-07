#!/usr/bin/env python3

# chmod u+x ~/catkin_ws/src/beginner_tutorials/src/explorer.py

import rospy
import math
import sys
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_msgs.msg import String
from turtlesim.srv import TeleportAbsolute, SetPen

class Explorer:
    def __init__(self, robot_id):
        self.robot_id = str(robot_id)
        self.nodename = "turtle" + self.robot_id
        
        rospy.init_node("explorer_node_" + self.robot_id, anonymous=True)
        
        self.pose = Pose()
        self.rate = rospy.Rate(60)
        self.targets = []
        self.found_targets = []
        self.all_found_targets = [] 
        
        self.total_distance = 0.0 
        self.my_turn = (self.robot_id == "1")
        self.start_direction = "DOWN" 
        
        self.robot_distances = {}
        self.leader_task_started = False
        self.target_sub_active = True
        
        rospy.wait_for_service('/' + self.nodename + '/teleport_absolute')
        rospy.wait_for_service('/' + self.nodename + '/set_pen')
        self.teleport_service = rospy.ServiceProxy('/' + self.nodename + '/teleport_absolute', TeleportAbsolute)
        self.set_pen_service = rospy.ServiceProxy('/' + self.nodename + '/set_pen', SetPen)

        self.vel_pub = rospy.Publisher('/' + self.nodename + '/cmd_vel', Twist, queue_size=10)
        self.info_pub = rospy.Publisher("/robot_info", String, queue_size=10, latch=True)

        self.pose_sub = rospy.Subscriber('/' + self.nodename + '/pose', Pose, self.update_pose)
        self.target_sub = rospy.Subscriber("/target_coordinates", String, self.target_callback)
        self.info_sub = rospy.Subscriber("/robot_info", String, self.info_callback)

    def update_pose(self, data):
        self.pose = data

    def target_callback(self, msg):
        coordinate_pairs = msg.data.split(";")
        for pair in coordinate_pairs:
            if pair != "":
                x_str, y_str = pair.split(",")
                self.targets.append((float(x_str), float(y_str)))
        
        if self.target_sub_active == True:
            self.target_sub.unregister()
            self.target_sub_active = False
            
        rospy.loginfo("[Turtle" + self.robot_id + "] Targets saved from generator.")

    def info_callback(self, msg):
        parts = msg.data.split("|")
        sender_id = parts[0]
        sender_dist = float(parts[4])
        
        self.robot_distances[sender_id] = sender_dist
        
        if len(parts) > 5 and parts[5] != "":
            found_pairs = parts[5].split(";")
            for pair in found_pairs:
                if pair != "":
                    x_str, y_str = pair.split(",")
                    parsed_t = (float(x_str), float(y_str))
                    if parsed_t not in self.all_found_targets:
                        self.all_found_targets.append(parsed_t)
        
        if (self.robot_id == "2" and sender_id == "1") or (self.robot_id == "3" and sender_id == "2"):
            last_x = float(parts[1])
            last_y = float(parts[2])
            last_dir = parts[3]
            
            start_x = last_x + 1.0
            start_y = last_y
            
            if last_dir == "DOWN":
                self.start_direction = "UP"
                start_theta = 1.57
            else:
                self.start_direction = "DOWN"
                start_theta = 4.71
            
            self.set_pen_service(255, 255, 255, 2, 1) 
            self.teleport_service(start_x, start_y, start_theta) 
            
            rospy.loginfo("[Turtle" + self.robot_id + "] It's my turn! Teleported to X=%.2f, Y=%.2f." % (start_x, start_y, ))
            self.my_turn = True
            
        self.check_for_leader()

    def check_targets(self):
        for t in self.targets:
            if t not in self.found_targets and t not in self.all_found_targets:
                dist = math.sqrt((t[0] - self.pose.x)**2 + (t[1] - self.pose.y)**2)
                if dist <= 1.0:
                    self.found_targets.append(t)
                    self.all_found_targets.append(t)
                    rospy.loginfo("[Turtle" + self.robot_id + "] Target Found! X=%.2f, Y=%.2f" % (t[0], t[1]))

    def rotate_robot(self, angle_degrees, clockwise=True):
        vel_msg = Twist()
        angular_speed = 0.5 
        if clockwise:
            vel_msg.angular.z = -abs(angular_speed)
        else:
            vel_msg.angular.z = abs(angular_speed)
        
        relative_angle = math.radians(abs(angle_degrees))
        t0 = rospy.Time.now().to_sec()
        current_angle = 0
        while current_angle < relative_angle:
            self.vel_pub.publish(vel_msg)
            t1 = rospy.Time.now().to_sec()
            current_angle = abs(angular_speed) * (t1 - t0)
            self.rate.sleep()
            
        vel_msg.angular.z = 0
        self.vel_pub.publish(vel_msg)
        rospy.sleep(0.1) 

    def move_side(self, speed=1.0): 
        vel_msg = Twist()
        vel_msg.linear.x = abs(speed)
        t0 = rospy.Time.now().to_sec()
        current_distance = 0
        while current_distance < 1.0: 
            self.check_targets() 
            self.vel_pub.publish(vel_msg)
            t1 = rospy.Time.now().to_sec()
            current_distance = abs(speed) * (t1 - t0) 
            self.rate.sleep()
        
        self.total_distance += 1.0 
        vel_msg.linear.x = 0
        self.vel_pub.publish(vel_msg)
        rospy.sleep(0.1) 

    def move_straight(self, target_y, speed=1.5):
        vel_msg = Twist()
        vel_msg.linear.x = speed
        t0 = rospy.Time.now().to_sec()
        
        while True: 
            self.check_targets() 
            self.vel_pub.publish(vel_msg)
            
            if abs(self.pose.y - target_y) <= 0.1:
                break
                
            self.rate.sleep()
            
        t1 = rospy.Time.now().to_sec()
        self.total_distance += abs(speed) * (t1 - t0) 
        
        vel_msg.linear.x = 0
        self.vel_pub.publish(vel_msg)
        rospy.sleep(0.1)

    def publish_my_info(self):
        targets_str = ""
        for t in self.all_found_targets:
            targets_str += "%.2f,%.2f;" % (t[0], t[1])
            
        last_dir = self.start_direction
        
        msg = "%s|%.2f|%.2f|%s|%.2f|%s" % (self.robot_id, self.pose.x, self.pose.y, last_dir, self.total_distance, targets_str)
        self.info_pub.publish(msg)
        
        self.robot_distances[self.robot_id] = self.total_distance
        rospy.loginfo("[Turtle" + self.robot_id + "] Finished! Distance: %.2f. Info sent to others." % self.total_distance)
        self.check_for_leader()


    def euclidean_distance(self, goal_pose):
        return math.sqrt(math.pow((goal_pose.x - self.pose.x), 2) + math.pow((goal_pose.y - self.pose.y), 2))

    def linear_vel(self, goal_pose, constant=1.5):
        hiz = constant * self.euclidean_distance(goal_pose)
        if hiz > 2.0:
            return 2.0
        return hiz

    def steering_angle(self, goal_pose):
        return math.atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)

    def angular_vel(self, goal_pose, constant=6):
        angle_error = self.steering_angle(goal_pose) - self.pose.theta
        
        while angle_error > math.pi:
            angle_error -= 2.0 * math.pi
        while angle_error < -math.pi:
            angle_error += 2.0 * math.pi
            
        return constant * angle_error

    def move_to_goal(self, target_x, target_y):
        goal_pose = Pose()
        goal_pose.x = target_x
        goal_pose.y = target_y
        
        tolerance = 0.1
        vel_msg = Twist()
        
        while self.euclidean_distance(goal_pose) >= tolerance:
            vel_msg.linear.x = self.linear_vel(goal_pose)
            vel_msg.angular.z = self.angular_vel(goal_pose)
            
            self.pub_to_move(vel_msg)
            
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.vel_pub.publish(vel_msg)
        rospy.sleep(0.5)

    def pub_to_move(self, vel_msg):
        self.vel_pub.publish(vel_msg)
        self.rate.sleep()



    def calculate_tsp_path(self):
        unvisited = []
        for t in self.all_found_targets:
            unvisited.append(t)
            
        current_pos = (self.pose.x, self.pose.y)
        path = []
        
        while len(unvisited) > 0:
            shortest_dist = 999999.0
            nearest_target = unvisited[0]
            
            for t in unvisited:
                dist = math.sqrt((t[0] - current_pos[0])**2 + (t[1] - current_pos[1])**2)
                if dist < shortest_dist:
                    shortest_dist = dist
                    nearest_target = t
                    
            path.append(nearest_target)
            current_pos = nearest_target
            unvisited.remove(nearest_target)
            
        return path

    def check_for_leader(self):
        if len(self.robot_distances) == 3 and self.leader_task_started == False:
            self.leader_task_started = True
            
            min_dist = 999999.0
            leader_id = "0"
            
            for i in self.robot_distances:
                if self.robot_distances[i] < min_dist:
                    min_dist = self.robot_distances[i]
                    leader_id = i
                    
            rospy.loginfo("[Turtle" + self.robot_id + "] All distances calculated. The LEADER is Turtle" + str(leader_id) + "!")
            
            if self.robot_id == leader_id:
                rospy.loginfo("[Turtle" + self.robot_id + "] I AM THE LEADER! Starting TSP Navigation...")
                tsp_path = self.calculate_tsp_path()
                
                self.set_pen_service(0, 255, 0, 3, 0)
                
                target_count = 1
                for t in tsp_path:
                    rospy.loginfo("[Turtle" + self.robot_id + "] Navigating to Target %d: X=%.2f, Y=%.2f" % (target_count, t[0], t[1]))
                    self.move_to_goal(t[0], t[1]) 
                    target_count += 1
                
                rospy.loginfo("[Turtle" + self.robot_id + "] ALL TARGETS VISITED. MISSION COMPLETELY ACCOMPLISHED!")

    def start_exploration(self):
        self.set_pen_service(255, 255, 255, 2, 1) 
        rospy.loginfo("[Turtle" + self.robot_id + "] Waiting for targets and my turn...")
        
        while (len(self.targets) == 0 or self.my_turn == False):
            self.rate.sleep()
        
        if self.robot_id == "1":
            self.teleport_service(1.5, 9.5, 4.71) 
            rospy.sleep(0.5)
            rospy.loginfo("[Turtle1] Teleported to the far left to cover all targets!")
            
        while len(self.found_targets) < 2 and len(self.all_found_targets) < 6:
            if self.start_direction == "DOWN":
                self.move_straight(target_y=1.5)
                if len(self.found_targets) >= 2 or len(self.all_found_targets) >= 6: 
                    break
                
                self.rotate_robot(90, clockwise=False)
                self.move_side()
                self.rotate_robot(90, clockwise=False)
                self.start_direction = "UP"
            else:
                self.move_straight(target_y=9.5)
                if len(self.found_targets) >= 2 or len(self.all_found_targets) >= 6: 
                    break
                
                self.rotate_robot(90, clockwise=True)
                self.move_side()
                self.rotate_robot(90, clockwise=True)
                self.start_direction = "DOWN"

        self.publish_my_info()

if __name__ == "__main__":
    if len(sys.argv) < 2:
        sys.exit(1)
        
    try:
        robot_id = sys.argv[1]
        exp = Explorer(robot_id)
        rospy.sleep(1.0) 
        exp.start_exploration()
        rospy.spin() 
    except rospy.ROSInterruptException:
        pass
