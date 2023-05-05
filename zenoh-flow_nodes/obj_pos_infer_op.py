from zenoh_flow.interfaces import Operator
from zenoh_flow import Input, Output
from zenoh_flow.types import Context
from typing import Dict, Any

import yaml

from geometry_msgs.msg import PoseStamped

import sys, os, inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
sys.path.insert(0, currentdir)
from comms_utils import *



class PathsPlanner(Operator):
    def __init__(
        self,
        context: Context,
        configuration: Dict[str, Any],
        inputs: Dict[str, Input],
        outputs: Dict[str, Output],
    ):
        configuration = {} if configuration is None else configuration

        self.input_obj_detected = inputs.get("ObjDetected", None)
        
        self.output_world_pos = outputs.get("WorldPosition", None)

        #Add the common configuration to this node's configuration
        common_cfg_file = str(configuration.get("common_cfg_file",
                                                "config/common_cfg.yaml"))
        common_cfg_yaml_file = open(common_cfg_file)
        common_cfg_dict = yaml.load(common_cfg_yaml_file, Loader=yaml.FullLoader)
        common_cfg_yaml_file.close()
        configuration.update(common_cfg_dict)

        #self.robot_namespaces = list(configuration.get("robot_namespaces",
        #                                               ["robot1", "robot2"]))
        self.ns_bytes_lenght = int(configuration.get("ns_bytes_lenght", 64))
        self.int_bytes_lenght = int(configuration.get("int_bytes_lenght", 4))

    async def iteration(self) -> None:

        data_msg = await self.input_obj_detected.recv()
        print("object detected", data_msg.data)
        
        ser_ns = data_msg.data[:self.ns_bytes_lenght]
        #ns = deser_string(ser_ns)
        #print(ns)

        #TODO: transform position from image to world coordinates.

        #TODO: serialize new world position.
        new_world_pos = [1.0, 1.0] #To test.
        new_world_pos = PoseStamped()
        new_world_pos.pose.position.x = 1.0
        new_world_pos.pose.position.y = 1.0
        ser_world_pos = ser_ros2_msg(new_world_pos)
        await self.output_world_pos.send(ser_ns + ser_world_pos)

        return None

    def finalize(self) -> None:
        return None



def register():
    return PathsPlanner
