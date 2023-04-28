from zenoh_flow.interfaces import Operator
from zenoh_flow import Input, Output
from zenoh_flow.types import Context
from typing import Dict, Any

import yaml, asyncio, cv2
from math import pi

from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.type_support import check_for_type_support

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point32, Point, Pose
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration

import sys, os, inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
sys.path.insert(0, currentdir)
from math_utils import *

#from rclpy.clock import Clock
from rclpy.time import Time



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
        self.output_markers = outputs.get("Markers", None)

        #Add the common configuration to this node's configuration
        common_cfg_file = str(configuration.get("common_cfg_file", "config/common_cfg.yaml"))
        common_cfg_yaml_file = open(common_cfg_file)
        common_cfg_dict = yaml.load(common_cfg_yaml_file, Loader=yaml.FullLoader)
        common_cfg_yaml_file.close()
        configuration.update(common_cfg_dict)

        self.robot_namespaces = list(configuration.get("robot_namespaces", ["robot1", "robot2"]))
        self.ns_bytes_lenght = int(configuration.get("ns_bytes_lenght", 64))
        self.int_bytes_lenght = int(configuration.get("int_bytes_lenght", 4))
        self.wp_separation = float(configuration.get("waypoint_separation", 1.0))

        #Map img interpr getting:
        self.map_yaml_file = str(configuration.get("map_yaml_file", "maps/turtlebot3_world/turtlebot3_world.yaml"))
        map_yaml_file = open(self.map_yaml_file)
        self.map_data_dict = yaml.load(map_yaml_file, Loader=yaml.FullLoader)
        map_yaml_file.close()
        self.map_img = cv2.imread(self.map_data_dict.get("image"), cv2.IMREAD_GRAYSCALE)
        map_negate = bool(self.map_data_dict.get("negate"))
        self.map_free_thresh = self.map_data_dict.get("free_thresh")
        self.map_occupied_thresh = self.map_data_dict.get("occupied_thresh")
        self.img_interpr = self.map_img / 255.0 if map_negate else (255 - self.map_img) / 255.0

        self.map_msg = OccupancyGrid()
        #self.wp_world_separation = 20
        self.resolution = int()
        self.origin = Point()

        self.pending = list()
        self.ids = 0
        self.times = 0
        self.colors = [[1.0, 0.5, 1.0, 1.0],
                       [0.0, 1.0, 0.5, 1.0]]

        check_for_type_support(OccupancyGrid)
        check_for_type_support(Pose)
        check_for_type_support(MarkerArray)

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

    def map2world(self, map_pose: Pose) -> Pose:
        world_pose = Pose()
        world_pose.position.x = (map_pose.position.x + self.origin.x) * self.resolution
        world_pose.position.y = (map_pose.position.y + self.origin.y) * self.resolution
        world_pose.orientation = map_pose.orientation
        return world_pose

    def img2map(self, img_pix: tuple, img_size: tuple) -> tuple:
        img_x, img_y = img_pix
        img_width, img_height = img_size
        map_pos = (float(img_x - (img_width/2)),
                   float(-(img_y - (img_height/2))))
        return map_pos

    def is_near_wall(self, point, margin, occupied_thresh):
        x, y = point
        width, height = self.img_interpr.shape
        for i in range(max(x - margin, 0), min(x + margin, width)):
            for j in range(max(y - margin, 0), min(y + margin, height)):
                if self.img_interpr[j, i] > occupied_thresh:
                    return True
        return False

    def get_path_from_area(self, p1, p2):
        path = list()
        vertical_range = list(range(int(p1.y), int(p2.y), round(self.wp_world_separation)))
        orientations = [(0, 0, 3*pi/2), (0, 0, pi/2)]
        ori_index = 0
        new_img = self.map_img
        for x in range(int(p1.x), int(p2.x), round(self.wp_world_separation)):
            for y in vertical_range:
                map_x, map_y = self.img2map((x, y), (self.map_msg.info.width, self.map_msg.info.height))
                wp = Pose()
                wp.position.x = map_x
                wp.position.y = map_y
                wp.orientation = euler2quat(orientations[ori_index%len(orientations)])
                
                #cv2.circle(new_img, (x, y), 2, 0, -1) #DEBUG
                if (self.img_interpr[y, x] < self.map_free_thresh and
                    not self.is_near_wall((x, y), 3, self.map_occupied_thresh)):
                    path.append(self.map2world(wp))
            vertical_range.reverse()
            ori_index += 1
        #cv2.imwrite("/tmp/test_points_map_img.png", new_img) #DEBUG
        return path

    def serialize_path(self, path):
        path_ser = bytes()
        for waypoint in path: #Serializing every waypoint:
            path_ser += _rclpy.rclpy_serialize(waypoint, type(waypoint))
        return path_ser

    def get_markers_from_path(self, path, ns):
        marker_array = MarkerArray()
        marker_array.markers = []
        for i, wp in enumerate(path):
            color = self.colors[self.times % len(self.colors)]
            merker_dict = {"id": self.ids+i, "ns": ns, "type": Marker.ARROW,
                            "frame_locked": False,
                            "lifetime": Duration(sec=0, nanosec=0),
                            "position": [wp.position.x, wp.position.y, 0.0],
                            "orientation": wp.orientation,
                            "scale": [0.1, 0.05, 0.05], "color_rgba": color}
            self.ids += 1
            marker_array.markers.append(get_marker(merker_dict))
        self.times += 1
        marker_array.markers[0].color.r = 1.0
        marker_array.markers[0].color.g = 0.0
        marker_array.markers[0].color.b = 0.0
        return marker_array

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
                self.map_msg = _rclpy.rclpy_deserialize(data_msg.data[p32_hsize*2:], OccupancyGrid)
                self.resolution = self.map_msg.info.resolution
                self.wp_world_separation = round(self.wp_separation / self.resolution)
                self.origin = self.map_msg.info.origin.position

                print("PATHS_PLANNER_OP -> map received, resolution:",
                      self.resolution, (self.origin.x, self.origin.y))
                
            if who == "Divisions":
                ns_bytes = data_msg.data[:self.ns_bytes_lenght]
                ns_from_bytes = str(ns_bytes.decode('utf-8')).lstrip()
                #We need to get the original name because in the decoded there
                #are empty bytes (/x00) that can't be removed otherwise:
                ns = self.robot_namespaces[self.robot_namespaces.index(ns_from_bytes)]

                point1, point2 = self.get_area_from_msg(data_msg.data)
                print("PATHS_PLANNER_OP -> div message received:", point1, point2)

                path = self.get_path_from_area(point1, point2)
                marker_array_msg = self.get_markers_from_path(path, ns)
                ser_markers = _rclpy.rclpy_serialize(marker_array_msg, type(marker_array_msg))
                await self.output_markers.send(ser_markers)
                
                path_len_ser = len(path).to_bytes(self.int_bytes_lenght, 'little')
                path_bytes = self.serialize_path(path)

                #print("PATHS_PLANNER_OP -> sending path:", path)
                await self.output_paths.send(ns_bytes + path_len_ser + path_bytes)

        return None

    def finalize(self) -> None:
        return None



def get_marker(def_dict):
    marker = Marker()
    #The main difference between them is that:
    marker.header.stamp = Time().to_msg() #this is the latest available transform in the buffer
    #marker.header.stamp = Clock().now().to_msg()# but this fetches the frame at the exact moment ((from rclpy.clock import Clock).
    marker.header.frame_id = "map"
    marker.ns = def_dict["ns"]
    marker.id = def_dict["id"]
    marker.frame_locked = def_dict["frame_locked"]
    marker.type = def_dict["type"]
    marker.action = Marker.ADD
    marker.lifetime = def_dict["lifetime"]
    marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = def_dict["position"]
    marker.pose.orientation = def_dict["orientation"]
    marker.scale.x, marker.scale.y, marker.scale.z = def_dict["scale"]
    marker.color.r, marker.color.g, marker.color.b, marker.color.a = def_dict["color_rgba"]
    return marker

def register():
    return PathsPlanner
