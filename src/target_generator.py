#!/usr/bin/env python3

# chmod u+x ~/catkin_ws/src/beginner_tutorials/src/target_generator.py

import rospy
import math
import random
from geometry_msgs.msg import Twist
from turtlesim.srv import TeleportAbsolute, SetPen
from std_msgs.msg import String

rospy.init_node("target_generator", anonymous=True)
pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)

target_pub = rospy.Publisher("/target_coordinates", String, queue_size=10, latch=True) 

rate = rospy.Rate(60)

rospy.wait_for_service('/turtle1/teleport_absolute')
rospy.wait_for_service('/turtle1/set_pen')

teleport_service = rospy.ServiceProxy('/turtle1/teleport_absolute', TeleportAbsolute)
set_pen_service = rospy.ServiceProxy('/turtle1/set_pen', SetPen)

def moveDistance(distance, isforward, speed):
    vel_msg = Twist()
    if isforward:
        vel_msg.linear.x = abs(speed)
    else:
        vel_msg.linear.x = -abs(speed)
    
    t0 = rospy.Time.now().to_sec()
    current_distance = 0
    while current_distance < distance:
        pub.publish(vel_msg)
        t1 = rospy.Time.now().to_sec()
        current_distance = abs(speed) * (t1 - t0) 
        rate.sleep()
        
    vel_msg.linear.x = 0
    pub.publish(vel_msg)

def rotate_robot(angle_degrees, clockwise=True):
    vel_msg = Twist()
    angular_speed = 5.0 

    if clockwise:
        vel_msg.angular.z = -abs(angular_speed)
    else:
        vel_msg.angular.z = abs(angular_speed)

    relative_angle = math.radians(abs(angle_degrees))
    t0 = rospy.Time.now().to_sec()
    current_angle = 0

    while current_angle < relative_angle:
        pub.publish(vel_msg)
        t1 = rospy.Time.now().to_sec()
        current_angle = abs(angular_speed) * (t1 - t0)
        rate.sleep()

    vel_msg.angular.z = 0
    pub.publish(vel_msg)

def draw_square():
    for i in range(4):
        moveDistance(0.5, True, 8.0) 
        rotate_robot(90, clockwise=False)

def generate_targets():
    rospy.loginfo("Targets are starting to be drawn...")
    
    valid_coordinates = [] 
    safe_distance = 1.5 
    
    target_count = 0
    

    while target_count < 6:
        random_x = random.uniform(1.5, 9.5)
        random_y = random.uniform(1.5, 9.5)
        
        is_safe = True
        
        for coord in valid_coordinates:
            dist = math.sqrt(math.pow((random_x - coord[0]), 2) + math.pow((random_y - coord[1]), 2))
            if dist < safe_distance:
                is_safe = False
                break 
                
        if is_safe == True:
            valid_coordinates.append((random_x, random_y))
            target_count += 1
    
    target_index = 1
    target_string = ""
    
    for coord in valid_coordinates:
        x_val = coord[0]
        y_val = coord[1]
        
        target_string += "%.2f,%.2f;" % (x_val, y_val)
        
        set_pen_service(255, 0, 0, 2, 1)
        teleport_service(x_val, y_val, 0)
        rospy.sleep(0.1)
        
        set_pen_service(255, 0, 0, 2, 0)
        rospy.sleep(0.2)
        
        draw_square()
        rospy.loginfo("Target %d placed: X=%.2f, Y=%.2f" % (target_index, x_val, y_val))
        target_index += 1

    rospy.loginfo("All targets have been successfully created!")

    target_pub.publish(target_string) 
    rospy.loginfo("Target coordinates published to /target_coordinates topic!")

    set_pen_service(255, 255, 255, 2, 1) 
    teleport_service(4.5, 10.0, 4.71)    
    rospy.loginfo("Turtle1 is at the starting position, ready for exploration!")
    
    rospy.spin() 

if __name__ == "__main__":
    try:
        generate_targets()
    except rospy.ROSInterruptException:
        pass
