from zenoh_flow.interfaces import Operator
from zenoh_flow import Input, Output
from zenoh_flow.types import Context
from typing import Dict, Any

import yaml

from visualization_msgs.msg import MarkerArray

import sys, os, inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
sys.path.insert(0, currentdir)
from math_utils import *
from comms_utils import *
from marker_utils import *
from map_utils import *



class PathsPlanner(Operator):
    def __init__(
        self,
        context: Context,
        configuration: Dict[str, Any],
        inputs: Dict[str, Input],
        outputs: Dict[str, Output],
    ):
        configuration = {} if configuration is None else configuration

        self.input_wp_req = inputs.get("WPRequest", None)
        
        self.output_debug_img = outputs.get("DebugImage", None)
        self.output_markers = outputs.get("Markers", None)
        
        self.output_next_wp = outputs.get("NextWP", None)

        #Add the common configuration to this node's configuration
        common_cfg_file = str(configuration.get("common_cfg_file",
                                                "config/common_cfg.yaml"))
        common_cfg_yaml_file = open(common_cfg_file)
        common_cfg_dict = yaml.load(common_cfg_yaml_file, Loader=yaml.FullLoader)
        common_cfg_yaml_file.close()
        configuration.update(common_cfg_dict)

        self.robot_namespaces = list(configuration.get("robot_namespaces",
                                                       ["robot1", "robot2"]))
        self.ns_bytes_lenght = int(configuration.get("ns_bytes_lenght", 64))
        self.int_bytes_lenght = int(configuration.get("int_bytes_lenght", 4))
        self.wp_separation = float(configuration.get("waypoint_separation", 1.0))

        self.debug_img_sent = False
        
        self.ids = 0
        self.times = 0
        self.colors = [[1.0, 0.5, 1.0, 1.0],
                       [0.0, 1.0, 0.5, 1.0]]

        self.pending = list()

        #Load map (yaml fields and img from yaml file):
        map_yaml_path = str(configuration.get(
                                "map_yaml_file",
                                "maps/turtlebot3_world/turtlebot3_world.yaml")
                            )
        self.load_map(map_yaml_path)
        #Get the bounding box and divide the map (get the bounding corners):
        divs, self.debug_div_img_msg_ser = divide_map(self.img_interpr,
                                                      self.map_img,
                                                      len(self.robot_namespaces),
                                                      (self.map_free_thresh,
                                                       self.map_occupied_thresh),
                                                      True)
        #For each division get the path and store it:
        marker_array_msg = MarkerArray()
        marker_array_msg.markers = []
        self.paths = []
        invert = False
        for div, ns in zip(divs, self.robot_namespaces):
            path = get_path_from_area(div, self.img_interpr,
                                      (self.map_free_thresh,
                                       self.map_occupied_thresh),
                                      self.wp_world_separation, self.map_origin,
                                      self.map_resolution, invert)
            invert = not invert
            self.paths.append(path)
            marker_array_msg.markers += self.get_markers_from_path(path, ns)
        self.marker_array_msg_ser = ser_ros2_msg(marker_array_msg)
        
    def load_map(self, map_yaml_path: str) -> None:
        map_yaml_file = open(map_yaml_path)
        self.map_data_dict = yaml.load(map_yaml_file, Loader=yaml.FullLoader)
        map_yaml_file.close()

        #Check the needed fields:
        keys_needed = ["image", "resolution", "origin",
                       "occupied_thresh", "free_thresh"]
        if (not all(map(lambda x: x in self.map_data_dict.keys(), keys_needed))\
            or len(self.map_data_dict.get("origin")) != 3): #[x, y, yaw]
            print(f"ERROR: required field/s ({keys_needed}) missing in map yaml file")
            raise Exception("ERROR: required field/s missing")
        
        self.map_img = cv2.imread(self.map_data_dict.get("image"),
                                  cv2.IMREAD_GRAYSCALE)
        #Map img values interpretation (Following the specifications in:
        #http://wiki.ros.org/map_server:
        map_negate = bool(self.map_data_dict.get("negate"))
        self.img_interpr = self.map_img / 255.0 if map_negate else (255 - self.map_img) / 255.0

        self.map_free_thresh = self.map_data_dict.get("free_thresh")
        self.map_occupied_thresh = self.map_data_dict.get("occupied_thresh")
        self.map_origin = self.map_data_dict.get("origin")
        self.map_resolution = self.map_data_dict.get("resolution")
        self.wp_world_separation = round(self.wp_separation / self.map_resolution)

    def get_markers_from_path(self, path: list, ns: str) -> list:
        markers = []
        for wp in path:
            self.ids += 1
            color = self.colors[self.times % len(self.colors)]
            marker_dict = {"id": self.ids, "ns": ns,
                            "frame_locked": False, "frame_id":"map",
                            "lifetime_s": 0, "lifetime_ns":0,
                            "pose": [wp.pose.position.x,
                                     wp.pose.position.y,
                                     wp.pose.orientation], # [x, y, yaw(quat)]
                            "scale": [0.1, 0.05, 0.05], "color_rgba": color}
            
            markers.append(get_marker(marker_dict))
        self.times += 1
        #The first one will always be red:
        markers[0].color.r = 1.0
        markers[0].color.g = 0.0
        markers[0].color.b = 0.0
        return markers


    async def iteration(self) -> None:

        if not self.debug_img_sent:
            await self.output_debug_img.send(self.debug_div_img_msg_ser)
            await self.output_markers.send(self.marker_array_msg_ser)
            
            #Send first wp for every path:
            for path, ns in zip(self.paths, self.robot_namespaces):
                ns_ser = ser_string(ns, self.ns_bytes_lenght, ' ')
                next_wp = path.pop(0)
                next_wp_ser = ser_ros2_msg(next_wp)
                await self.output_next_wp.send(ns_ser + next_wp_ser)

            self.debug_img_sent = True

        data_msg = await self.input_wp_req.recv()

        ns = deser_string(data_msg.data, ' ') #data_msg.data is the ns serialized.
        index = self.robot_namespaces.index(ns)
        next_wp = self.paths[index].pop(0)
        next_wp_ser = ser_ros2_msg(next_wp)
        await self.output_next_wp.send(data_msg.data + next_wp_ser)

        return None

    def finalize(self) -> None:
        return None



def register():
    return PathsPlanner
