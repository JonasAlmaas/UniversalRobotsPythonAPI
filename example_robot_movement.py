import ur_api as ur


class Pose:
    pos1 = ur.Pose(x= 860, y=-216, z=1130, rx=2.4, ry=-1.7, rz=2.6)
    pos2 = ur.Pose(x= 860, y=-216, z=800, rx=2.4, ry=-1.7, rz=2.6)


robot = ur.UniversalRobot()

robot.set_accel(1.5)
robot.set_vel(1.5)

print("Before movement 1")
robot.move_to(target_pose=Pose.pos1, wait=True)
print("After movment 2")

print("Before movement 2")
robot.move_to(target_pose=Pose.pos2, wait=False)
print("After movment 2")

# robot.set_forcemove(state=True)
