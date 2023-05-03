from zenoh_flow.interfaces import Operator
from zenoh_flow import Input, Output
from zenoh_flow.types import Context
from typing import Dict, Any

import yaml, cv2, numpy as np
from math import pi

from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.type_support import check_for_type_support

from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Duration

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from visualization_msgs.msg import Marker, MarkerArray
#from rclpy.clock import Clock
from rclpy.time import Time

import sys, os, inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
sys.path.insert(0, currentdir)
from math_utils import *
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

        self.input_wp_req = inputs.get("WPRequest", None)
        
        self.output_debug_img = outputs.get("DebugImage", None)
        self.output_markers = outputs.get("Markers", None)
        
        self.output_next_wp = outputs.get("NextWP", None)

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

        self.cv_bridge = CvBridge()
        self.debug_img_sent = False
        
        self.ids = 0
        self.times = 0
        self.colors = [[1.0, 0.5, 1.0, 1.0],
                       [0.0, 1.0, 0.5, 1.0]]

        self.pending = list()

        check_for_type_support(PoseStamped)
        check_for_type_support(Image)
        check_for_type_support(MarkerArray)

        #Load map (yaml fields and img from yaml file):
        map_yaml_path = str(configuration.get("map_yaml_file", "maps/turtlebot3_world/turtlebot3_world.yaml"))
        self.load_map(map_yaml_path)
        #Get the bounding box and divide the map (get the bounding corners):
        divs = self.divide_map()
        #For each division get the path and store it:
        marker_array_msg = MarkerArray()
        marker_array_msg.markers = []
        self.paths = []
        invert = False
        for div, ns in zip(divs, self.robot_namespaces):
            path = self.get_path_from_area(div, invert)
            invert = not invert
            self.paths.append(path)
            marker_array_msg.markers += self.get_markers_from_path(path, ns)
        self.marker_array_msg_ser = _rclpy.rclpy_serialize(marker_array_msg, type(marker_array_msg))
        
        
    def load_map(self, map_yaml_path):
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
        #Map img values interpretation (Following the specifications in
        #http://wiki.ros.org/map_server:
        map_negate = bool(self.map_data_dict.get("negate"))
        self.img_interpr = self.map_img / 255.0 if map_negate else (255 - self.map_img) / 255.0

        self.map_free_thresh = self.map_data_dict.get("free_thresh")
        self.map_occupied_thresh = self.map_data_dict.get("occupied_thresh")
        self.map_origin = self.map_data_dict.get("origin")
        self.map_resolution = self.map_data_dict.get("resolution")
        self.wp_world_separation = round(self.wp_separation / self.map_resolution)
        

    def factorize(self, n):
        prime_numbers = [2, 3, 5, 7, 11, 13, 17, 19, 23, 29, 31, 37, 41,
                         43, 47, 53, 59, 61, 67, 71, 73, 79, 83, 89, 97]
        factors = []
        if n < 2:
            return factors
        
        for p in prime_numbers:
            while n % p == 0:
                factors.append(p)
                n /= p
            if (n == 1):
                break
        return factors
    
    def get_squarest_distribution(self, factors):
        distribution = [1, 1]
        for f in reversed(factors): #Starts from the highest to the lowest.
            if distribution[0] < distribution[1]:
                distribution[0] *= f #The factor is multiplied by the lowest member of the distribution
            else:
                distribution[1] *= f
        return distribution

    def get_division_shape(self):
        map_parts = len(self.robot_namespaces)
        factors = self.factorize(map_parts)

        if len(factors) == 0: #For only one robot
            return [1, 1]
        elif len(factors) == 1:
            factors.append(1)
            return factors
        elif len(factors) == 2:
            return factors
        else: #More than one single configuration
            return self.get_squarest_distribution(factors)

    def get_map_upper_bound(self) -> int:
        height, width = self.img_interpr.shape
        for j in range(height):
            for i in range(width):
                if not (self.map_free_thresh < self.img_interpr[j, i] < self.map_occupied_thresh):
                    return j
                
    def get_map_lower_bound(self) -> int:
        height, width = self.img_interpr.shape
        for j in range(height-1, 0, -1):
            for i in range(width):
                if not (self.map_free_thresh < self.img_interpr[j, i] < self.map_occupied_thresh):
                    return j

    def get_map_left_bound(self) -> int:
        height, width = self.img_interpr.shape
        for i in range(width):
            for j in range(height):
                if not (self.map_free_thresh < self.img_interpr[j, i] < self.map_occupied_thresh):
                    return i

    def get_map_right_bound(self) -> int:
        height, width = self.img_interpr.shape
        for i in range(width-1, 0, -1):
            for j in range(height):
                if not (self.map_free_thresh < self.img_interpr[j, i] < self.map_occupied_thresh):
                    return i
        
    def divide_map(self):
        left_top_point = [
            self.get_map_left_bound(),
            self.get_map_upper_bound()]
        right_bot_point = [
            self.get_map_right_bound(),
            self.get_map_lower_bound()]
        reduced_width = right_bot_point[0] - left_top_point[0]
        reduced_height = right_bot_point[1] - left_top_point[1]

        division_shape = self.get_division_shape()
        height, width = self.img_interpr.shape
        if reduced_width > reduced_height:
            if division_shape[0] > division_shape[1]:
                division_shape = list(reversed(division_shape))
        else:
            if division_shape[0] < division_shape[1]:
                division_shape = list(reversed(division_shape))
        print(f"MAP_PATHS_PLANNER_OP -> Map division shape: {division_shape}")

        x_shift = reduced_width / division_shape[0]
        y_shift = reduced_height / division_shape[1]
        bboxes = [] #Map divided into N bboxes:
        for i in range(division_shape[0]):
            for j in range(division_shape[1]):
                bboxes.append([(round(left_top_point[0] + x_shift * i),
                                round(left_top_point[1] + y_shift * j)),
                               (round(left_top_point[0] + x_shift * (i+1)),
                                round(left_top_point[1] + y_shift * (j+1)))])

        ### TO DEBUG:
        debug_div_img = np.array(self.map_img) #Copy the img.
        for i in range(division_shape[0] + 1):
            cv2.line(debug_div_img,
                     [round(left_top_point[0] + (x_shift*i)),
                      round(left_top_point[1])],
                     [round(left_top_point[0] + (x_shift*i)),
                      round(left_top_point[1] + reduced_height)],
                     0, 1)
        for j in range(division_shape[1] + 1):
            cv2.line(debug_div_img,
                     [round(left_top_point[0]),
                      round(left_top_point[1] + y_shift*j)],
                     [round(left_top_point[0] + reduced_width),
                      round(left_top_point[1] + y_shift*j)],
                     0, 1)
        for bbox in bboxes:
            for point in bbox:
                cv2.circle(debug_div_img, point, 2, 0, 2, -1)
        for i in range(division_shape[0] + 1):
            for j in range(division_shape[1] + 1):
                cv2.circle(debug_div_img,
                           (round(left_top_point[0] + (x_shift*i)), round(left_top_point[1] + y_shift*j)),
                           2, 0, 2, -1)
        debug_img_msg = self.cv_bridge.cv2_to_imgmsg(debug_div_img)
        self.debug_div_img_msg_ser = _rclpy.rclpy_serialize(debug_img_msg, type(debug_img_msg))
        ###

        return bboxes

    def map2world(self, map_pose: PoseStamped) -> PoseStamped:
        world_pose = PoseStamped()
        world_pose.pose.position.x = (map_pose.pose.position.x +
                                      self.map_origin[0]) * self.map_resolution
        world_pose.pose.position.y = (map_pose.pose.position.y +
                                      self.map_origin[1]) * self.map_resolution
        world_pose.pose.orientation = map_pose.pose.orientation
        return world_pose

    def img2map(self, img_pix: tuple) -> tuple:
        img_x, img_y = img_pix
        img_height, img_width = self.map_img.shape
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

    def get_path_from_area(self, area, inverted=False):
        p1, p2 = area
        path = list()
        vertical_range = list(range(int(p1[1]),
                                    int(p2[1]),
                                    round(self.wp_world_separation)))
        orientations = [(0, 0, 3*pi/2), (0, 0, pi/2)]
        if inverted:
            vertical_range.reverse()
        ori_index = 0
        #new_img = self.map_img #DEBUG
        for x in range(int(p1[0]), int(p2[0]), round(self.wp_world_separation)):
            for y in vertical_range:
                wp = PoseStamped()
                wp.header.frame_id = "map"
                wp.header.stamp.sec = 0
                wp.header.stamp.nanosec = 0
                wp.pose.position.x, wp.pose.position.y = self.img2map((x, y))
                wp.pose.orientation = euler2quat(
                    orientations[ori_index % len(orientations)]
                    )
                
                #cv2.circle(new_img, (x, y), 2, 0, -1) #DEBUG
                if (self.img_interpr[y, x] < self.map_free_thresh and
                    not self.is_near_wall((x, y), 3, self.map_occupied_thresh)):
                    path.append(self.map2world(wp))
            vertical_range.reverse()
            ori_index += 1
        #cv2.imwrite("/tmp/test_points_map_img.png", new_img) #DEBUG
        if inverted:
            path.reverse()
        return path

    def get_markers_from_path(self, path, ns):
        markers = []
        for wp in path:
            self.ids += 1
            color = self.colors[self.times % len(self.colors)]
            marker_dict = {"id": self.ids, "ns": ns, "type": Marker.ARROW,
                            "frame_locked": False,
                            "lifetime": Duration(sec=0, nanosec=0),
                            "position": [wp.pose.position.x, wp.pose.position.y, 0.0],
                            "orientation": wp.pose.orientation,
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
                next_wp_ser = _rclpy.rclpy_serialize(next_wp, type(next_wp))
                await self.output_next_wp.send(ns_ser + next_wp_ser)

            self.debug_img_sent = True

        data_msg = await self.input_wp_req.recv()

        ns = deser_string(data_msg.data, ' ') #data_msg.data is the ns serialized.
        index = self.robot_namespaces.index(ns)
        next_wp = self.paths[index].pop(0)
        next_wp_ser = _rclpy.rclpy_serialize(next_wp, type(next_wp))
        await self.output_next_wp.send(data_msg.data + next_wp_ser)

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
