import robot

import numpy as np
import math

# gain constants
import sys

from params import *

IP_FORMAT = "192.168.1.%d"

inbox = {}

# Phi value for each robot at time t-1.
old_phi = np.zeros(n)


def on_data_received(robot_id, src_add, dst_add, port, data):
    global inbox
    # global n
    # n = len(inbox)
    # print "robots:", n
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
        next = (robot_id + 1) % n
        next_ip = IP_FORMAT % ((robot_id + 2) % (n + 1)) if (robot_id + 2) % (n + 1) != 0 else  IP_FORMAT % 1
        # robot.send_to(robot_id, str(phi), next_ip)
        # # Before in ring
        before = (robot_id - 1) % n
        # before_ip = IP_FORMAT % (robot_id % (n + 1))
        before_ip = IP_FORMAT % (robot_id % (n + 1)) if robot_id != 0 else  IP_FORMAT % n

        # robot.send_to(robot_id, str(phi), before_ip)
        robot_ip = IP_FORMAT % ((robot_id + 1) % (n + 1))
        on_data_received(next, robot_ip, next_ip, 55, str(phi))
        on_data_received(before, robot_ip, before_ip, 55, str(phi))

        # print robot_id, inbox

        # No inbox
        if robot_id not in inbox:
            print "Warning: no inbox for", robot_id
            return
        if before_ip not in inbox[robot_id] or next_ip not in inbox[robot_id]:
            print "Warning: no data from neighbors for", robot_id, "neighbours:", inbox[robot_id], (before_ip, next_ip)
            return

        if inbox[robot_id][before_ip] is None or inbox[robot_id][next_ip] is None:
            print "Warning: missed neighbor for", robot_id
            return


        ############### Controller ##############
        # Average Phi from neighbors
        back1 = float(inbox[robot_id][before_ip])
        front1 = float(inbox[robot_id][next_ip])
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



        # Feedback linearization
        d = .1
        vi = dot_x * math.cos(rrz) + dot_y * math.sin(rrz)
        wi = - dot_x * math.sin(rrz) / d + dot_y * math.cos(rrz) / d

        robot.set_linear_velocity(robot_id, vi, 0, 0)
        robot.set_angular_velocity(robot_id, 0, 0, wi)


        ### Clear inbox
        # inbox[robot_id][before_ip] = None
        # inbox[robot_id][next_ip] = None

    except:
        print "Unexpected error:", sys.exc_info()
        raise
