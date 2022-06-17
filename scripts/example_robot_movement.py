import os
import sys

sys.path.insert(1, os.path.abspath('.'))
from urpy import urpy


class Pose:
    pose1 = urpy.Pose(x=1060, y=-216, z=800, rx=-1.44, ry=1.02, rz=-1.56)
    pose2 = urpy.Pose(x= 860, y=-216, z=1130, rx=2.4, ry=-1.7, rz=2.6)
    # camera = urpy.Pose(1003, -180, 1244, -1.48, 1.0, -1.51)
    # camera = urpy.Pose(853, -180, 1220, -1.48, 1.0, -1.51)
    camera = urpy.Pose(x=870.0, y=-181.7, z=1215.4, rx=-1.49, ry=1.01, rz=-1.5)
    
    pmi_start_1 = urpy.Pose(x=659, y=-174, z=678, rx=-1.28, ry=1.21, rz=-1.17)
    pmi_start_2 = urpy.Pose(x=1084, y=-161, z=486, rx=1.97, ry=-1.91, rz=-0.47)


class JointPosition:
    example1 = urpy.JointPosition(base=0.19357706606388092, shoulder=-2.1495567760863246, elbow=-1.497639536857605, wrist1=-0.28987331808123784, wrist2=0.49242764711380005, wrist3=-0.395120922719137)
    example2 = urpy.JointPosition(base=-0.615, shoulder=-2.839, elbow=-0.416, wrist1=-1.013, wrist2=2.589, wrist3=-0.395)
    example3 = urpy.JointPosition(base=0.523, shoulder=-2.514, elbow=-0.84, wrist1=-0.06, wrist2=1.556, wrist3=-0.395)

robot = urpy.UniversalRobot("192.168.1.101")

robot.move_to(target_pose=Pose.camera)

robot.move_to_joint_pos(target=JointPosition.example1)
robot.move_to_joint_pos(target=JointPosition.example2)
robot.move_to_joint_pos(target=JointPosition.example3)

# robot.move_to(target_pose=Pose.pmi_start_1)

# robot.move_to(target_pose=Pose.pose2)
# robot.move_to(target_pose=Pose.pose1)

# robot.move_to(urpy.Pose(x=680, y=-200, z=400, rx=2.59, ry=-1.77, rz=0))
