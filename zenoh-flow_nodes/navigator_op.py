from zenoh_flow.interfaces import Operator
from zenoh_flow import Input, Output
from zenoh_flow.types import Context
from typing import Dict, Any

import yaml

from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.type_support import check_for_type_support

from geometry_msgs.msg import Point32



class Navigator(Operator):
    def __init__(
        self,
        context: Context,
        configuration: Dict[str, Any],
        inputs: Dict[str, Input],
        outputs: Dict[str, Output],
    ):
        configuration = {} if configuration is None else configuration

        self.input_paths = inputs.get("Paths", None)
        self.output_wps = outputs.get("Waypoint", None)

        #Add the common configuration to this node's configuration
        common_cfg_file = str(configuration.get("common_cfg_file", "config/common_cfg.yaml"))
        common_cfg_yaml_file = open(common_cfg_file)
        common_cfg_dict = yaml.load(common_cfg_yaml_file, Loader=yaml.FullLoader)
        common_cfg_yaml_file.close()
        configuration.update(common_cfg_dict)

        self.ns_bytes_lenght = int(configuration.get("ns_bytes_lenght", 64))

        self.pending = list()

        check_for_type_support(Point32)

    async def iteration(self) -> None:
        data_msg = await self.input_paths.recv()

        ns_bytes = data_msg.data[:self.ns_bytes_lenght]
        path_len_bytes = data_msg.data[self.ns_bytes_lenght:self.ns_bytes_lenght*2]
        path_bytes = data_msg.data[self.ns_bytes_lenght*2:]
        print("NAVIGATOR_OP -> path received")

        #TODO: create robotPath objects for every namespace to store their paths:
        path_len = int.from_bytes(path_len_bytes, 'little')
        ns = ns_bytes.decode('utf-8')
        print(f"NAVIGATOR_OP -> {ns} path_len: {path_len}")
        path = []
        p32_hsize = len(_rclpy.rclpy_serialize(Point32(), Point32)) # 16
        for i in range(0, path_len*p32_hsize, p32_hsize):
            path.append(_rclpy.rclpy_deserialize(path_bytes[i:i + p32_hsize], Point32))
            print(f"NAVIGATOR_OP -> {ns} point: {path[-1]}")



    def finalize(self) -> None:
        return None



def register():
    return Navigator
