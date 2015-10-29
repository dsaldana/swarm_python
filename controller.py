import robot


def on_data_received(robot_id, src_add, dst_add, port, data):
    print "received!!!", robot_id, src_add, dst_add, port, data


def update(robot_id):
    #print "updating", robot_id
    robot.set_linear_velocity(robot_id, 1, 0, 0)
    robot.set_angular_velocity(robot_id, 0, 0, .1)
    
    #print robot.neighbors(robot_id)
    #print "pose:", robot.pose(robot_id)
    #print "search_area:", robot.search_area(robot_id)
    #print "camera",robot_id,":", robot.camera(robot_id)
    
    for neigh in robot.neighbors(robot_id):
        print "->", neigh, robot.send_to(robot_id, "hi ", neigh)
    print "-------"
