import ur_api as ur


class Pose:
    pos1 = ur.Pose(x= 860, y=-216, z=1130, rx=2.4, ry=-1.7, rz=2.6)
    pos2 = ur.Pose(x= 860, y=-216, z=800, rx=2.4, ry=-1.7, rz=2.6)

    camera = ur.Pose(1003, -180, 1244, -1.48, 1.0, -1.51)

robot = ur.UniversalRobot()

# robot.move_to(target_pose=Pose.pos1, wait=True)
robot.move_to(target_pose=Pose.camera, wait=True)

# robot.set_forcemove(state=True)
