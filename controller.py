import robot

import time
import math


def on_data_received(robot_id, src_add, dst_add, port, data):
    #print "received!!!", robot_id, src_add, dst_add, port, data
    pass


def update(robot_id):
    #print "updating", robot_id
    #robot.set_linear_velocity(robot_id, 1, 0, 0)
    #robot.set_angular_velocity(robot_id, 0, 0, .1)
    
    #print robot.neighbors(robot_id)
    #print "pose:", robot_id, robot.pose(robot_id)
    #print "imu:", robot_id, robot.imu(robot_id)
    print "bearing:", robot_id, robot.bearing(robot_id)
    
    #print "search_area:", robot.search_area(robot_id)
    #print "camera",robot_id,":", robot.camera(robot_id)
    
    #for neigh in robot.neighbors(robot_id):
        #print "->", neigh, robot.send_to(robot_id, "hi ", neigh)
    #    robot.send_to(robot_id, "hi ", neigh)
    #print "-------"
    
    # Get pose
def bla():
    rx, ry, rh = robot.pose(robot_id)

    t = time.time()
    
    # Virtual robot
    #x = 2 * math.cos(.1*t + robot_id * math.pi) + 35
    #y = 2 * math.cos(.1*t + robot_id * math.pi) + 120
    
    x,y = 35, 120
    

    d = math.sqrt((rx - x)**2 + (ry - y)**2)
    ang = math.atan2((y-ry),(x-rx))
   
    robot.set_linear_velocity(robot_id, .0 * d, 0, 0)
    robot.set_angular_velocity(robot_id, 0, 0, .1 * ang)
    
    print robot_id, d, ang
    
