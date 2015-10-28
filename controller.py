import robot

#print "hello", robot.set_linear_velocity(1,0,0)




def update(robot_id):
    #print "updating"
    #print "fd", robot_id
#    print "as", 
    
    robot.set_linear_velocity(robot_id, 1, 0, 0)
    robot.set_angular_velocity(robot_id, 0, 0, .1)
    
    #print "m=",multiply(2,3)
    return
