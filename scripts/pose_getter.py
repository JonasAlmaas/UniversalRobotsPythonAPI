# Middle mouse to get pose
# Left mouse to end program
import os
import sys

from pynput.mouse import Listener, Button

p = os.path.abspath('.')
sys.path.insert(1, p)

import urpy as urpy

robot = urpy.UniversalRobot()

robot.set_freedrive(state=True)

def on_mouse_click(x, y, button, pressed):
    if pressed:
        if button == Button.middle:
            pose = robot.get_pose()
            print("Pose(x=" + str(pose.x) + ", y=" + str(pose.y) + ", z=" + str(pose.z) + ", rx=" + str(pose.rx) + ", ry=" + str(pose.ry) + ", rz" + str(pose.rz) + ")")
        
        if button == Button.left:
            robot.set_freedrive(state=False)
            return False

with Listener(on_click=on_mouse_click) as listener:
    listener.join()
