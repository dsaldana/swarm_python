import robot

import numpy as np
import math

# gain constants
kp, kphi = .1, .05

# Desired radius
R = .5
# Desired angular speed
Omeg = 1.1

inbox = {}

n = 2


old_phi = np.zeros(n)

def on_data_received(robot_id, src_add, dst_add, port, data):
    global inbox
    # print "new data"
    if robot_id not in inbox:
        inbox[robot_id] = {}
    if src_add not in inbox[robot_id]:
        inbox[robot_id][src_add] = None

    inbox[robot_id][src_add] = data


def update(robot_id):
    # Get pose
    # print "1"
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

    # Next in ring
    next = "192.168.1.%d" % ((robot_id + 1) % n + 1)
    robot.send_to(robot_id, str(phi), next)
    # last in ring
    last = "192.168.1.%d" % ((robot_id - 1) % n + 1)
    robot.send_to(robot_id, str(phi), last)


    # print "inbox", inbox
    # No inbox
    if robot_id not in inbox:
        print "Warning: no inbox for", robot_id
        return
    if last not in inbox[robot_id] or next not in inbox[robot_id]:
        print "Warning: no data from neighbors for", robot_id
        return



    # Average Phi from neighbors
    back1 = float(inbox[robot_id][last])
    front1 = float(inbox[robot_id][next])
    phi_av = (back1 + front1) / 2.

    if robot_id == 0:
        phi_av -= math.pi
    if robot_id == n - 1:
        phi_av += math.pi

    # Controller
    dot_rho = kp * (R - rho)
    dot_phi = Omeg + kphi * (phi_av - phi)
    #dot_phi = kphi * (math.pi * robot_id - phi)

    dot_x = dot_rho * math.cos(phi) - rho * dot_phi * math.sin(phi)
    dot_y = dot_rho * math.sin(phi) + rho * dot_phi * math.cos(phi)

    if robot_id == 1:
        print rho
    # print robot_id, dot_phi
    #print robot_id, next, last
    # Tangent vector


    # Feedback linearization. Equation (12)
    # d = .2
    # v = wx * math.cos(rtheta) + wy * math.sin(rtheta)
    # ang = - wx * math.sin(rtheta) / d + wy * math.cos(rtheta) / d

    robot.set_linear_velocity(robot_id, dot_x, dot_y, 0)
    # robot.set_angular_velocity(robot_id, 0, 0, ang)

    # print "vels: ", robot_id, v, ang, (rx, ry)
