import os
import sys

p = os.path.abspath('.')
sys.path.insert(1, p)

from urpy import urpy


class Pose:
    pose1 = urpy.Pose(x=1060, y=-216, z=800, rx=-1.44, ry=1.02, rz=-1.56)
    pose2 = urpy.Pose(x= 860, y=-216, z=1130, rx=2.4, ry=-1.7, rz=2.6)
    # camera = urpy.Pose(1003, -180, 1244, -1.48, 1.0, -1.51)
    camera = urpy.Pose(853, -180, 1140, -1.48, 1.0, -1.51)


robot = urpy.UniversalRobot("192.168.1.101")

robot.move_to(target_pose=Pose.camera)
# robot.move_to(target_pose=Pose.pose2)
# robot.move_to(target_pose=Pose.pose1)

# robot.move_to(urpy.Pose(x=680, y=-200, z=400, rx=2.59, ry=-1.77, rz=0))
