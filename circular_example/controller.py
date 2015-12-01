import robot

import numpy as np
import math

# gain constants
import sys

kp, kphi = 1.5, .3

# Desired radius
R = 4.
# Desired angular speed
Omeg = 0.5

inbox = {}

n = 36


old_phi = np.zeros(n)


def on_data_received(robot_id, src_add, dst_add, port, data):
    global inbox
    # global n
    # n = len(inbox)
    # print "new data"
    if robot_id not in inbox:
        inbox[robot_id] = {}
    if src_add not in inbox[robot_id]:
        inbox[robot_id][src_add] = None

    inbox[robot_id][src_add] = data


def update(robot_id):
    try:
        global old_phi
        ########## Get pose #####################
        # print robot_id, "1"
        rx, ry, rz, rrx, rry, rrz = robot.gazebo_pose(robot_id)

        rho = math.sqrt(rx ** 2 + ry ** 2)
        phi = math.atan2(ry, rx)

        # delta phi
        dphi = math.atan2(math.sin(phi - old_phi[robot_id]), math.cos(phi - old_phi[robot_id]))
        phi = old_phi[robot_id] + dphi
        old_phi[robot_id] = phi

        # Share location
        # for neigh in robot.neighbors(robot_id):
        #     if neigh == "boo":
        #         continue
        #
        #     print "->", neigh, robot.send_to(robot_id, "hi ", neigh)
        #     robot.send_to(robot_id, str(phi), neigh)
        #     print "fff"

        ############# Send location to the neighbors ##############
        # Next in ring
        next = "192.168.1.%d" % ((robot_id + 1) % n + 1)
        robot.send_to(robot_id, str(phi), next)
        # Before in ring
        before = "192.168.1.%d" % ((robot_id - 1) % n + 1)
        robot.send_to(robot_id, str(phi), before)

        # No inbox
        if robot_id not in inbox:
            print "Warning: no inbox for", robot_id
            return
        if before not in inbox[robot_id] or next not in inbox[robot_id]:
            print "Warning: no data from neighbors for", robot_id
            return


        ############### Controller ##############
        # Average Phi from neighbors
        back1 = float(inbox[robot_id][before])
        front1 = float(inbox[robot_id][next])
        phi_av = (back1 + front1) / 2.

        if robot_id == 0:
            phi_av -= math.pi
        if robot_id == n - 1:
            phi_av += math.pi

        # Control input
        dot_rho = kp * (R - rho)
        dot_phi = Omeg + kphi * (phi_av - phi)

        # Convert to cartesian coordinates
        dot_x = dot_rho * math.cos(phi) - rho * dot_phi * math.sin(phi)
        dot_y = dot_rho * math.sin(phi) + rho * dot_phi * math.cos(phi)

        if robot_id == 3:
            print rho


        # Feedback linearization. Equation (12)
        # d = .2
        # v = wx * math.cos(rtheta) + wy * math.sin(rtheta)
        # ang = - wx * math.sin(rtheta) / d + wy * math.cos(rtheta) / d

        robot.set_linear_velocity(robot_id, dot_x, dot_y, 0)
        # robot.set_angular_velocity(robot_id, 0, 0, ang)

    except:
        print "Unexpected error:", sys.exc_info()[0]
        raise
