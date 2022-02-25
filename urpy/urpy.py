from __future__ import annotations

import argparse
import socket
import time
import os

from enum import Enum, auto

import logging
from .rtde import rtde
from .rtde import rtde_config


PORT_SEND = 30002
PORT_RECEIVE = 30004
CONFIG = "configuration.xml"


class MovementType(Enum):
    '''Defined how the robot should move.'''
    LINEAR = auto()
    QUICKEST = auto()


class UniversalRobot:

    def __init__(self, host_ip: str) -> None:
        '''A helper class to comunicate with a Universal Robot.'''
        self._host: str = host_ip
        self._accel: float = 1.5
        self._vel: float = 1.5

        parser = argparse.ArgumentParser()
        parser.add_argument('--host', default=self._host,help='name of host to connect to (localhost)')
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

    def send_to_robot(self, function_str: str) -> None:
        '''Sends a function string to the robot.'''
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((self._host, PORT_SEND))
        s.send(function_str.encode())
        s.close()

    def set_accel(self, accel: float) -> None:
        '''Sets the acceleration for the robot.'''
        self._accel = accel
    
    def set_vel(self, vel: float) -> None:
        '''Sets the velcoity for the robot.'''
        self._vel = vel

    def move_to(self, target_pose: Pose, movement_type: MovementType = MovementType.QUICKEST, wait: bool = True) -> None:
        '''Moves the robot to the desiered pose, waits before continuing if the 'wait' flag is set.'''
        target_pose.to_m()

        if movement_type is MovementType.LINEAR:
            self.send_to_robot(target_pose.movel(self._accel, self._vel))
        elif movement_type is MovementType.QUICKEST:
            self.send_to_robot(target_pose.movej(self._accel, self._vel))
        else:
            print("[urpy][ERROR]: Unknown movement type.")
            return

        if wait:
            target_pose.to_mm()

            is_at_positon = False
            while not is_at_positon:
                current_pose = self.get_pose()
                is_at_positon = target_pose == current_pose
                if not is_at_positon:
                    time.sleep(0.1)

    def get_pose(self) -> Pose:
        '''Get the robots correct pose as an instance of the Pose class.'''
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
            print("[urpy][ERROR]: Failed to get pose!")

        con.send_pause()
        con.disconnect()

        pose = Pose(x, y, z, rx, ry, rz)
        pose.to_mm()

        return pose
    
    def set_freedrive(self, state=True) -> None:
        '''Sets the robot in freedrive mode until another request is sent.'''
        function_str = None

        if state:
            function_str = "def prog():\nfreedrive_mode()\nsleep(99999999)\nend\nprog()"
        else:
            function_str = "end_freedrive_mode()\n"

        self.send_to_robot(function_str)


class Pose:
    '''A wrapper around an arm pose.'''
    def __init__(self, x: float = 0.0, y: float = 0.0, z: float = 0.0, rx: float = 0.0, ry: float = 0.0, rz: float = 0.0) -> None:
        self.x: float = x
        self.y: float = y
        self.z: float = z
        self.rx: float = round(rx, 2)
        self.ry: float = round(ry, 2)
        self.rz: float = round(rz, 2)

    def to_m(self) -> None:
        '''Converts from mm to m.'''
        self.x = self.x / 1000.0
        self.y = self.y / 1000.0
        self.z = self.z / 1000.0
    
    def to_mm(self) -> None:
        '''Converts from m to mm.'''
        self.x = round(self.x * 1000.0, 1)
        self.y = round(self.y * 1000.0, 1)
        self.z = round(self.z * 1000.0, 1)

    def to_declaration(self) -> str:
        '''Returns the declaration of the pose.'''
        return "Pose(x=" + str(self.x) + ", y=" + str(self.y) + ", z=" + str(self.z) + ", rx=" + str(self.rx) + ", ry=" + str(self.ry) + ", rz=" + str(self.rz) + ")"

    def _get_undefined_move_command(self, a, v) -> str:
        '''Helper function for sending a pose to the robot.'''
        return "p[" + str(self.x) + ", " + str(self.y) + ", " + str(self.z) + ", " + str(self.rx) + ", " + str(self.ry) + ", " + str(self.rz) + "],a=" + str(a) + ", v=" +str(v)

    def movej(self, a, v) -> str:
        '''Returns a function string for the pose.'''
        return "movej(" + self._get_undefined_move_command(a, v) + ")\n"

    def movel(self, a, v) -> str:
        '''Returns a function string for the pose.'''
        return "movel(" + self._get_undefined_move_command(a, v) + ")\n"
    
    def lerp(self, other: Pose, percent: float) -> Pose:
        '''Returns a linear interpolated pose.'''
        x = lerp(self.x, other.x, percent)
        y = lerp(self.y, other.y, percent)
        z = lerp(self.z, other.z, percent)
        rx = lerp(self.rx, other.rx, percent)
        ry = lerp(self.ry, other.ry, percent)
        rz = lerp(self.rz, other.rz, percent)
        return Pose(x, y, z, rx, ry, rz)

    def copy(self) -> Pose:
        '''Returns a copy of the pose.'''
        return Pose(x=self.x, y=self.y, z=self.z, rx=self.rx, ry=self.ry, rz=self.rz)

    def __eq__(self, other):
        if (isinstance(other, Pose)):
            # return (self.x == other.x) and (self.y == other.y) and (self.z == other.z) and (self.rx == other.rx) and (self.ry == other.ry) and (self.rz == other.rz)
            return (self.x == other.x) and (self.y == other.y) and (self.z == other.z)
        return False
    
    def __str__(self):
        return "X: " + str(self.x) + ", Y: " + str(self.y) + ", Z: " + str(self.z) + ", rX: " + str(self.rx) + ", rY: " + str(self.ry) + ", rZ: " + str(self.rz)


def lerp(a: float, b: float, percent: float) -> float:
    '''Takes two values and a percent between 0 and 1. Returns a liner interpolated value.'''
    return a + ((b - a) * percent)
