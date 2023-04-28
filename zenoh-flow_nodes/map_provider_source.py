from zenoh_flow.interfaces import Source
from zenoh_flow import Output
from zenoh_flow.types import Context
from typing import Any, Dict

import yaml
import cv2, numpy as np

from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.type_support import check_for_type_support

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Quaternion, Point32



class MapProvider(Source):
    def __init__(
        self,
        context: Context,
        configuration: Dict[str, Any],
        outputs: Dict[str, Output],
    ):
        self.output_map = outputs.get("Map", None)
        
        configuration = {} if configuration is None else configuration
        self.map_yaml_file = str(configuration.get("map_yaml_file", "maps/turtlebot3_world/turtlebot3_world.yaml"))

        check_for_type_support(OccupancyGrid)
        check_for_type_support(Point32)

        #We only compute it once as the map is always the same:
        self.ser_map_msg = self.serialize_reduced_map_msg() #in bytes, prepared to be sent.
        self.sent = False


    def get_map_and_bbox(self) -> OccupancyGrid:
        #Open the map yaml file:
        map_yaml_file = open(self.map_yaml_file)

        #Get the data:
        map_data_dict = yaml.load(map_yaml_file, Loader=yaml.FullLoader)
        map_yaml_file.close()

        #Check the needed fields:
        keys_needed = ["image", "resolution", "origin",
                       "occupied_thresh", "free_thresh"]
        if (not all(map(lambda x: x in map_data_dict.keys(), keys_needed)) or\
                len(map_data_dict.get("origin")) != 3): #[x, y, yaw]
            print(f"ERROR: required field/s ({keys_needed}) missing in map yaml file")
            raise Exception("ERROR: required field/s missing")
        
        #Create the message obj and fill its fields:
        map_msg = OccupancyGrid()

        map_msg.header.frame_id = "map"
        #map_msg.header.stamp.sec = 0.0
        #map_msg.header.stamp.nanosec = 0.0
        map_msg.info.resolution = map_data_dict.get("resolution")
        #map_msg.info.map_load_time.sec = 0.0
        #map_msg.info.map_load_time.nanosec = 0.0
        
        if len(map_data_dict.get("origin")) == 3:
            map_msg.info.origin.position.x,\
            map_msg.info.origin.position.y,\
            yaw = list(map_data_dict.get("origin"))
            map_msg.info.origin.orientation = self.get_quaternion_from_euler([0.0, 0.0, yaw])

        map_img = cv2.imread(map_data_dict.get("image"), cv2.IMREAD_GRAYSCALE)
        
        bbox_points = self.get_bbox(map_img,
                        bool(map_data_dict.get("negate")),
                        map_data_dict.get("free_thresh"),
                        map_data_dict.get("occupied_thresh"))
        map_msg.info.width, map_msg.info.height = map_img.shape
        map_img.resize(map_msg.info.width * map_msg.info.height)
        map_msg.data = list(map(lambda x: int(x-128), map_img)) #Convert from [0, 255] to [-128, 127]

        return map_msg, bbox_points
    
    def get_quaternion_from_euler(self, rpy : list) -> Quaternion: # [roll, pitch, yaw]
        roll, pitch, yaw = rpy
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)

    def get_bbox(self, img: np.ndarray, negate: bool,
                lower_thresh: float, higher_thresh: float) -> tuple:
        #Following the specs of http://wiki.ros.org/map_server.
        img_interpr = img / 255.0 if negate else (255 - img) / 255.0
        
        left_top_point = Point32()
        left_top_point.x = float(self.get_map_left_bound(img_interpr, lower_thresh, higher_thresh))
        left_top_point.y = float(self.get_map_upper_bound(img_interpr, lower_thresh, higher_thresh))
        right_bot_point = Point32()
        right_bot_point.x = float(self.get_map_right_bound(img_interpr, lower_thresh, higher_thresh))
        right_bot_point.y = float(self.get_map_lower_bound(img_interpr, lower_thresh, higher_thresh))
        return (left_top_point, right_bot_point) #Diagonal points.


    def reduce_empty_space(self, img: np.ndarray, negate: bool,
                           lower_thresh: float, higher_thresh: float,
                           margin: int) -> np.ndarray:
        #Following the specs of http://wiki.ros.org/map_server.
        img_interpr = img / 255.0 if negate else (255 - img) / 255.0
        
        min_y = self.get_map_upper_bound(img_interpr, lower_thresh, higher_thresh)
        max_y = self.get_map_lower_bound(img_interpr, lower_thresh, higher_thresh)
        min_x = self.get_map_left_bound(img_interpr, lower_thresh, higher_thresh)
        max_x = self.get_map_right_bound(img_interpr, lower_thresh, higher_thresh)

        height, width = img.shape
        margin_min_x = max(min_x - margin, 0)
        margin_max_x = min(max_x + margin, width - 1)
        margin_min_y = max(min_y - margin, 0)
        margin_max_y = min(max_y + margin, height - 1)
        origin_pix_shift = (margin_min_x, height - margin_max_y)
        new_img = (255-img[margin_min_y:margin_max_y, margin_min_x:margin_max_x])*255

        return (origin_pix_shift, new_img)


    def get_map_upper_bound(self, img: np.ndarray,
                            lower_thresh: float, higher_thresh: float) -> int:
        height, width = img.shape
        for j in range(height):
            for i in range(width):
                if not (lower_thresh < img[j, i] < higher_thresh):
                    return j
                
    def get_map_lower_bound(self, img: np.ndarray,
                            lower_thresh: float, higher_thresh: float) -> int:
        height, width = img.shape
        for j in range(height-1, 0, -1):
            for i in range(width):
                if not (lower_thresh < img[j, i] < higher_thresh):
                    return j

    def get_map_left_bound(self, img: np.ndarray,
                            lower_thresh: float, higher_thresh: float) -> int:
        height, width = img.shape
        for i in range(width):
            for j in range(height):
                if not (lower_thresh < img[j, i] < higher_thresh):
                    return i

    def get_map_right_bound(self, img: np.ndarray,
                            lower_thresh: float, higher_thresh: float) -> int:
        height, width = img.shape
        for i in range(width-1, 0, -1):
            for j in range(height):
                if not (lower_thresh < img[j, i] < higher_thresh):
                    return i

    def serialize_reduced_map_msg(self) -> bytes:
        map_msg, bbox_points = self.get_map_and_bbox() #Get map message

        #Serialize the messages together:
        serialized_msgs = bytes()
        for point in bbox_points: # append all the bytes together
            serialized_msgs += _rclpy.rclpy_serialize(point, type(point))
        serialized_msgs += _rclpy.rclpy_serialize(map_msg, type(map_msg))
        return serialized_msgs

    async def iteration(self) -> None:
        if not self.sent:
            await self.output_map.send(self.ser_map_msg)
            print("MAP_PROVIDER_OP -> sending map and bbox message...")
            self.sent = True
        return None

    def finalize(self) -> None:
        return None



def register():
    return MapProvider