from zenoh_flow.interfaces import Operator
from zenoh_flow import Input, Output
from zenoh_flow.types import Context
from typing import Dict, Any

import yaml, asyncio

from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.type_support import check_for_type_support

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point32, Point



class PathsPlanner(Operator):
    def __init__(
        self,
        context: Context,
        configuration: Dict[str, Any],
        inputs: Dict[str, Input],
        outputs: Dict[str, Output],
    ):
        configuration = {} if configuration is None else configuration

        self.input_map = inputs.get("Map", None)
        self.input_divs = inputs.get("Divisions", None)
        self.output_paths = outputs.get("Paths", None)

        #Add the common configuration to this node's configuration
        common_cfg_file = str(configuration.get("common_cfg_file", "config/common_cfg.yaml"))
        common_cfg_yaml_file = open(common_cfg_file)
        common_cfg_dict = yaml.load(common_cfg_yaml_file, Loader=yaml.FullLoader)
        common_cfg_yaml_file.close()
        configuration.update(common_cfg_dict)

        self.ns_bytes_lenght = int(configuration.get("ns_bytes_lenght", 64))
        self.wp_separation = int(configuration.get("waypoint_separation", 64))
        self.resolution = int()
        self.origin = Point()

        self.pending = list()

        check_for_type_support(OccupancyGrid)
        check_for_type_support(Point32)

    def get_area_from_msg(self, msg_data: bytes) -> tuple():
        #We dont actually care about the namespaces, but this piece of code will be useful latter.
        #ns_bytes = msg_data[:self.ns_bytes_lenght]
        #ns = str()
        #for i, byte in enumerate(ns_bytes):
        #    if byte != bytes(1):
        #        ns = msg_data[i:self.ns_bytes_lenght].decode('utf-8')
        #        break
        #print("ns received:", ns)

        p32_hsize = len(_rclpy.rclpy_serialize(Point32(), Point32)) # 16
        points_bytes = msg_data[self.ns_bytes_lenght:]
        return (_rclpy.rclpy_deserialize(points_bytes[:p32_hsize], Point32),
                _rclpy.rclpy_deserialize(points_bytes[p32_hsize:p32_hsize*2], Point32))

    def get_path_between(self, p1, p2):
        path = list()
        rang = list(range(round(p1.y), round(p2.y), self.wp_separation))
        for x in range(round(p1.x), round(p2.x), self.wp_separation):
            for y in rang:
                wp = Point32()
                wp.x = float(x * self.resolution) + self.origin.x
                wp.y = float(y * self.resolution) + self.origin.y
                path.append(wp)
            rang.reverse()
        return path

    def serialize_path(self, path):
        path_ser = bytes()
        for waypoint in path: #Serialize every waypoint:
            path_ser += _rclpy.rclpy_serialize(waypoint, Point32)
        return path_ser

    async def wait_map(self):
        data_msg = await self.input_map.recv()
        return ("Map", data_msg)

    async def wait_divs(self):
        data_msg = await self.input_divs.recv()
        return ("Divisions", data_msg)

    def create_task_list(self):
        task_list = [] + self.pending

        if not any(t.get_name() == "Map" for t in task_list):
            task_list.append(
                asyncio.create_task(self.wait_map(), name="Map")
            )
        if not any(t.get_name() == "Divisions" for t in task_list):
            task_list.append(
                asyncio.create_task(self.wait_divs(), name="Divisions")
            )
        return task_list

    async def iteration(self) -> None:
        (done, pending) = await asyncio.wait(
            self.create_task_list(),
            return_when=asyncio.FIRST_COMPLETED,
        )
        self.pending = list(pending)
        for d in done:
            (who, data_msg) = d.result()
            if who == "Map":
                p32_hsize = len(_rclpy.rclpy_serialize(Point32(), Point32)) # 16
                map_msg = _rclpy.rclpy_deserialize(data_msg.data[p32_hsize*2:], OccupancyGrid)
                self.resolution = map_msg.info.resolution
                self.origin = map_msg.info.origin.position
                print("PATHS_PLANNER_OP -> map received, resolution:",
                      self.resolution, (self.origin.x, self.origin.y))
                
            if who == "Divisions":
                ns_bytes = data_msg.data[:self.ns_bytes_lenght]
                point1, point2 = self.get_area_from_msg(data_msg.data)
                print("PATHS_PLANNER_OP -> div message received:", point1, point2)

                path = self.get_path_between(point1, point2)
                #We use the same bytes size in self.ns_bytes_lenght:
                path_len_ser = len(path).to_bytes(self.ns_bytes_lenght, 'little')
                path_bytes = self.serialize_path(path)

                #print("PATHS_PLANNER_OP -> sending path:", path)
                await self.output_paths.send(ns_bytes + path_len_ser + path_bytes)

        return None

    def finalize(self) -> None:
        return None



def register():
    return PathsPlanner
