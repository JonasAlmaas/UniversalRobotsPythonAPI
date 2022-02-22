# Middle mouse to get pose
# Left mouse to end program

from pynput.mouse import Listener, Button

import ur_api as ur

robot = ur.UniversalRobot()
robot.set_freedrive(state=True)

def on_mouse_click(x, y, button, pressed):
    if pressed:
        if button == Button.middle:
            pose = robot.get_pose()
            print("Pose(" + str(pose.x) + ", " + str(pose.y) + ", " + str(pose.z) + ", " + str(pose.rx) + ", " + str(pose.ry) + ", " + str(pose.rz) + ")")
        
        if button == Button.left:
            robot.set_freedrive(state=False)
            return False

with Listener(on_click=on_mouse_click) as listener:
    listener.join()
