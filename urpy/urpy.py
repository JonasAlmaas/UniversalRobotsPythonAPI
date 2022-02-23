import argparse
import socket
import time
import os

import logging
from .rtde import rtde
from .rtde import rtde_config


HOST = "192.168.1.101"
PORT_SEND = 30002
PORT_RECEIVE = 30004
CONFIG = "configuration.xml"


class Pose:
    def __init__(self, x=0.0, y=0.0, z=0.0, rx=0.0, ry=0.0, rz=0.0):
        self.x = x
        self.y = y
        self.z = z
        self.rx = round(rx, 2)
        self.ry = round(ry, 2)
        self.rz = round(rz, 2)

    def to_m(self):
        '''Converts from m to mm.'''
        self.x = self.x / 1000.0
        self.y = self.y / 1000.0
        self.z = self.z / 1000.0
    
    def to_mm(self):
        '''Converts from mm to m.'''
        self.x = round(self.x * 1000.0)
        self.y = round(self.y * 1000.0)
        self.z = round(self.z * 1000.0)

    def to_declaration(self):
        '''Returns the declaration of the pose.'''
        return "Pose(x=" + str(self.x) + ", y=" + str(self.y) + ", z=" + str(self.z) + ", rx=" + str(self.rx) + ", ry=" + str(self.ry) + ", rz=" + str(self.rz) + ")"

    def movej(self, a, v) -> str:
        '''Returns a string for the pose.'''
        return "movej(p[" + str(self.x) + ", " + str(self.y) + ", " + str(self.z) + ", " + str(self.rx) + ", " + str(self.ry) + ", " + str(self.rz) + "],a=" + str(a) + ", v=" +str(v) + ")\n"

    def copy(self):
        return Pose(x=self.x, y=self.y, z=self.z, rx=self.rx, ry=self.ry, rz=self.rz)

    def __eq__(self, other):
        if (isinstance(other, Pose)):
            # return (self.x == other.x) and (self.y == other.y) and (self.z == other.z) and (self.rx == other.rx) and (self.ry == other.ry) and (self.rz == other.rz)
            return (self.x == other.x) and (self.y == other.y) and (self.z == other.z)
        return False
    
    def __str__(self):
        return "X: " + str(self.x) + ", Y: " + str(self.y) + ", Z: " + str(self.z) + ", rX: " + str(self.rx) + ", rY: " + str(self.ry) + ", rZ: " + str(self.rz)


class UniversalRobot:

    def __init__(self):
        self._accel = 1.5
        self._vel = 1.5

        parser = argparse.ArgumentParser()
        parser.add_argument('--host', default=HOST,help='name of host to connect to (localhost)')
        parser.add_argument('--port', type=int, default=PORT_RECEIVE, help='port number (30004)')
        parser.add_argument('--samples', type=int, default=0,help='number of samples to record')
        parser.add_argument('--frequency', type=int, default=125, help='the sampling frequency in Herz')
        parser.add_argument('--config', default=os.path.join(os.path.dirname(__file__), CONFIG), help='data configuration file to use (record_configuration.xml)')
        parser.add_argument("--verbose", help="increase output verbosity", action="store_true")
        parser.add_argument("--buffered", help="Use buffered receive which doesn't skip data", action="store_true")
        parser.add_argument("--binary", help="save the data in binary format", action="store_true")
        self._args = parser.parse_args()

        if self._args.verbose:
            logging.basicConfig(level=logging.INFO)

    def send_to_robot(self, function_str: str):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((HOST, PORT_SEND))
        s.send(function_str.encode())
        s.close()

    def set_accel(self, accel: float):
        '''Sets the acceleration for the robot.'''
        self._accel = accel
    
    def set_vel(self, vel: float):
        '''Sets the velcoity for the robot.'''
        self._vel = vel

    def move_to(self, target_pose: Pose, wait=True):
        '''Moves the robot to the desiered pose, waits before continuing if the 'wait' flag is set.'''
        target_pose.to_m()

        self.send_to_robot(target_pose.movej(self._accel, self._vel))

        if wait:
            target_pose.to_mm()

            is_at_positon = False
            while not is_at_positon:
                current_pose = self.get_pose()
                is_at_positon = target_pose == current_pose
                if not is_at_positon:
                    time.sleep(0.1)

    def get_pose(self) -> Pose:
        '''Get the robots correct pose as a struct. Example: pose.x, pose.rx'''
        conf = rtde_config.ConfigFile(self._args.config)
        output_names, output_types = conf.get_recipe('out')

        con = rtde.RTDE(self._args.host, self._args.port)
        con.connect()

        con.get_controller_version()
        con.send_output_setup(output_names, output_types, frequency=self._args.frequency)
        con.send_start()

        if self._args.buffered:
            state = con.receive_buffered(self._args.binary)
        else:
            state = con.receive(self._args.binary)

        if state is not None:
            x, y, z, rx, ry, rz = state.actual_TCP_pose
        else:
            print("Something went wrong!")

        con.send_pause()
        con.disconnect()

        pose = Pose(x, y, z, rx, ry, rz)
        pose.to_mm()

        return pose
    
    def set_freedrive(self, state=True):
        '''Sets the robot in freedrive mode until another request is sent.'''
        function_str = None

        if state:
            function_str = "def prog():\nfreedrive_mode()\nsleep(99999999)\nend\nprog()"
        else:
            function_str = "end_freedrive_mode()\n"

        self.send_to_robot(function_str)
    
    def set_forcemode(self, state):
        if state:
            pass
        else:
            pass
