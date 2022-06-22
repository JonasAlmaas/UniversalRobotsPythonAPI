import os
import sys

sys.path.insert(1, os.path.abspath('.'))

import urpy


class Pose:
    example1 = urpy.Pose(x=870.0, y=-181.7, z=1215.4, rx=-1.49, ry=1.01, rz=-1.5)
    example2 = urpy.Pose(x=700.0, y=-181.7, z=700.4, rx=-1.49, ry=1.01, rz=-1.5)


class JointPosition:
    example1 = urpy.JointPosition(base=0.19357706606388092, shoulder=-2.1495567760863246, elbow=-1.497639536857605, wrist1=-0.28987331808123784, wrist2=0.49242764711380005, wrist3=-0.395120922719137)
    example2 = urpy.JointPosition(base=-0.615, shoulder=-2.839, elbow=-0.416, wrist1=-1.013, wrist2=2.589, wrist3=-0.395)
    example3 = urpy.JointPosition(base=0.523, shoulder=-2.514, elbow=-0.84, wrist1=-0.06, wrist2=1.556, wrist3=-0.395)


robot = urpy.UniversalRobot("192.168.1.101")

robot.move_to(urpy.JointPosition(base=0.004, shoulder=-1.653, elbow=-1.247, wrist1=-1.78, wrist2=1.537, wrist3=-0.0))

robot.move_to(target=Pose.example1)
robot.move_to(target=Pose.example2)
robot.move_to(target=JointPosition.example1)
robot.move_to(target=JointPosition.example2)
robot.move_to(target=JointPosition.example3)

path = [
    urpy.PathPoint(Pose.example1),
    urpy.PathPoint(Pose.example2),
    urpy.PathPoint(JointPosition.example1),
    urpy.PathPoint(JointPosition.example2),
    urpy.PathPoint(JointPosition.example3)
]
robot.move_path(path_points=path)
