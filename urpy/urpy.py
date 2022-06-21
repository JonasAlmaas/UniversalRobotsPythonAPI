from __future__ import annotations

from enum import Enum, auto
from typing import Union, List

import os
import time
import socket
import argparse

import logging

from traitlets import Float
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
    '''A helper class to communicate with a Universal Robot.'''
    def __init__(self, host_ip: str) -> None:
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

    def set_accel(self, accel: float) -> None:
        '''Sets the acceleration for the robot.'''
        self._accel = accel
    
    def set_vel(self, vel: float) -> None:
        '''Sets the velcoity for the robot.'''
        self._vel = vel

    def set_freedrive(self, state=True) -> None:
        '''Sets the robot in freedrive mode until another request is sent.'''
        function_str = None

        if state:
            function_str = "def prog():\nfreedrive_mode()\nsleep(99999999)\nend\nprog()"
        else:
            function_str = "end_freedrive_mode()\n"

        self._send_to_robot(function_str)

    def _send_to_robot(self, function_str: str) -> None:
        '''Sends a function string to the robot.'''
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((self._host, PORT_SEND))
        s.send(function_str.encode())
        s.close() 

    def move_to_pose(self, target_pose: Pose, movement_type: MovementType = MovementType.QUICKEST, wait: bool = True) -> None:
        '''Moves the robot to the desiered pose, waits before continuing if the 'wait' flag is set.'''
        if movement_type is MovementType.LINEAR:
            self._send_to_robot(target_pose.movel(self._accel, self._vel))
        elif movement_type is MovementType.QUICKEST:
            self._send_to_robot(target_pose.movej(self._accel, self._vel))
        else:
            print("[urpy][ERROR]: Unknown movement type.")
            return

        if wait:
            is_at_positon = False
            while not is_at_positon:
                current_pose = self.get_pose()
                is_at_positon = target_pose == current_pose
                if not is_at_positon:
                    time.sleep(0.1)

    def move_to_joint_pos(self, target: JointPosition, wait: bool = True) -> None:
        '''Moves the robot to the desiered joint position, waits before continuing if the 'wait' flag is set.'''
        self._send_to_robot(target.movej(self._accel, self._vel))

        if wait:
            is_at_positon = False
            while not is_at_positon:
                current_pos = self.get_joint_position()
                is_at_positon = target == current_pos
                if not is_at_positon:
                    time.sleep(0.1)

    def move_path(self, path_points: List[PathPoint], wait: bool = True) -> None:
        '''Given an array of PathPoints the robot moves smoothly between them with little delay. Waits before continuing.'''
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((self._host, PORT_SEND))

        function_str: str = "def path():\n"

        # Create the function string
        for point in path_points:
            if isinstance(point.target, JointPosition):
                function_str = function_str + point.target.movej(self._accel, self._vel)
            else:
                if point.movement_type == MovementType.QUICKEST:
                    function_str = function_str + point.target.movej(self._accel, self._vel)
                elif point.movement_type == MovementType.LINEAR:
                    function_str = function_str + point.target.movel(self._accel, self._vel)
                else:
                    print("[urpy][ERROR]: Unknown movement type.")

        function_str += "end\n"     # End the script
        function_str += "path()"    # Run the script

        s.send(function_str.encode())
        s.close()

        # Wait for the robot to reach the last path position
        if wait:
            target = path_points[len(path_points) - 1].target
            move_done = False
            while not move_done:
                if isinstance(target, JointPosition):
                    move_done = target == self.get_joint_position()
                else:
                    move_done = target == self.get_pose()

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
    
    def get_joint_position(self) -> JointPosition:
        '''Get the robots correct joint positions as an instance of the JointPosition class.'''
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
            base, shoulder, elbow, wrist1, wrist2, wrist3 = state.actual_q
        else:
            print("[urpy][ERROR]: Failed to get joint position!")

        con.send_pause()
        con.disconnect()

        joint_position = JointPosition(base, shoulder, elbow, wrist1, wrist2, wrist3)

        return joint_position


class Pose:
    '''A wrapper around an arm pose.'''
    def __init__(self, x: float = 0.0, y: float = 0.0, z: float = 0.0, rx: float = 0.0, ry: float = 0.0, rz: float = 0.0) -> None:
        self.x: float = x
        self.y: float = y
        self.z: float = z
        self.rx: float = round(rx, 2)
        self.ry: float = round(ry, 2)
        self.rz: float = round(rz, 2)

    def _get_in_m(self):
        '''Returns pose in m. The robot needs meter space cordinates.'''
        x = self.x / 1000.0
        y = self.y / 1000.0
        z = self.z / 1000.0
        return x, y, z

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
        x, y, z = self._get_in_m()
        return "p[" + str(x) + ", " + str(y) + ", " + str(z) + ", " + str(self.rx) + ", " + str(self.ry) + ", " + str(self.rz) + "],a=" + str(a) + ", v=" +str(v)

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

    def _kinda_equal(self, other: Pose, threshold: Float):
        '''Returns true if the arm is within a threshold of other.'''
        if isinstance(other, Pose):
            x = (self.x + threshold >= other.x) and (self.x - threshold <= other.x)
            y = (self.y + threshold >= other.y) and (self.y - threshold <= other.y)
            z = (self.z + threshold >= other.z) and (self.z - threshold <= other.z)
            return x and y and z
        return False

    def __eq__(self, other: Pose):
        if isinstance(other, Pose):
            # return (self.x == other.x) and (self.y == other.y) and (self.z == other.z) and (self.rx == other.rx) and (self.ry == other.ry) and (self.rz == other.rz)
            return (self.x == other.x) and (self.y == other.y) and (self.z == other.z)
        return False
    
    def __str__(self):
        return "X: " + str(self.x) + ", Y: " + str(self.y) + ", Z: " + str(self.z) + ", rX: " + str(self.rx) + ", rY: " + str(self.ry) + ", rZ: " + str(self.rz)


class JointPosition:
    '''A wrapper around a joint position. Defined in radians!'''
    def __init__(self, base: float, shoulder: float, elbow: float, wrist1: float, wrist2: float, wrist3: float) -> None:
        self.base: float = round(base, 3)
        self.shoulder: float = round(shoulder, 3)
        self.elbow: float = round(elbow, 3)
        self.wrist1: float = round(wrist1, 3)
        self.wrist2: float = round(wrist2, 3)
        self.wrist3: float = round(wrist3, 3)

    def to_declaration(self) -> str:
        '''Returns the declaration of the joint position.'''
        return "JointPosition(base=" + str(self.base) + ", shoulder=" + str(self.shoulder) + ", elbow=" + str(self.elbow) + ", wrist1=" + str(self.wrist1) + ", wrist2=" + str(self.wrist2) + ", wrist3=" + str(self.wrist3) + ")"

    def _get_undefined_move_command(self, a, v) -> str:
        '''Helper function for sending a pose to the robot.'''
        return "[" + str(self.base) + ", " + str(self.shoulder) + ", " + str(self.elbow) + ", " + str(self.wrist1) + ", " + str(self.wrist2) + ", " + str(self.wrist3) + "],a=" + str(a) + ", v=" +str(v)

    def movej(self, a, v) -> str:
        '''Returns a function string for the joint position.'''
        return "movej(" + self._get_undefined_move_command(a, v) + ")\n"

    def lerp(self, other: JointPosition, percent: float) -> Pose:
        '''Returns a linear interpolated pose.'''
        base = lerp(self.base, other.base, percent)
        shoulder = lerp(self.shoulder, other.shoulder, percent)
        elbow = lerp(self.elbow, other.elbow, percent)
        wrist1 = lerp(self.wrist1, other.wrist1, percent)
        wrist2 = lerp(self.wrist2, other.wrist2, percent)
        wrist3 = lerp(self.wrist3, other.wrist3, percent)
        return Pose(base, shoulder, elbow, wrist1, wrist2, wrist3)

    def copy(self) -> JointPosition:
        '''Returns a copy of the joint position.'''
        return JointPosition(self.base, self.shoulder, self.elbow, self.wrist1, self.wrist2, self.wrist3)

    def _kinda_equal(self, other: JointPosition, threshold: Float):
        '''Returns true if the arm is within a threshold of other.'''
        if isinstance(other, JointPosition):
            base = (self.base + threshold >= other.base) and (self.base - threshold <= other.base)
            shoulder = (self.shoulder + threshold >= other.shoulder) and (self.shoulder - threshold <= other.shoulder)
            elbow = (self.elbow + threshold >= other.elbow) and (self.elbow - threshold <= other.elbow)
            wrist1 = (self.wrist1 + threshold >= other.wrist1) and (self.wrist1 - threshold <= other.wrist1)
            wrist2 = (self.wrist2 + threshold >= other.wrist2) and (self.wrist2 - threshold <= other.wrist2)
            wrist3 = (self.wrist3 + threshold >= other.wrist3) and (self.wrist3 - threshold <= other.wrist3)
            return base and shoulder and elbow and wrist1 and wrist2 and wrist3
        return False

    def __eq__(self, other):
        if (isinstance(other, JointPosition)):
            return (self.base == other.base) and (self.shoulder == other.shoulder) and (self.elbow == other.elbow) and (self.wrist1 == other.wrist1) and (self.wrist2 == other.wrist2) and (self.wrist3 == other.wrist3)
        return False
    
    def __str__(self):
        return "Base: " + str(self.base) + ", Shoulder: " + str(self.shoulder) + ", Elbow: " + str(self.elbow) + ", Wrist1: " + str(self.wrist1) + ", Wrist2: " + str(self.wrist2) + ", Wrist3: " + str(self.wrist3)


class PathPoint:
    def __init__(self, target: Union[Pose, JointPosition], movement_type: MovementType = MovementType.QUICKEST) -> None:
        self.target: Union[Pose, JointPosition] = target
        self.movement_type: MovementType = movement_type


def lerp(a: float, b: float, percent: float) -> float:
    '''Takes two values and a percent between 0 and 1. Returns a liner interpolated value.'''
    return a + ((b - a) * percent)
