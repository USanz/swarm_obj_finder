from zenoh_flow.interfaces import Operator
from zenoh_flow import Input, Output
from zenoh_flow.types import Context
from typing import Dict, Any

import yaml
from math import floor
import cv2, numpy as np

from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.type_support import check_for_type_support

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point32



class MapDivider(Operator):
    def __init__(
        self,
        context: Context,
        configuration: Dict[str, Any],
        inputs: Dict[str, Input],
        outputs: Dict[str, Output],
    ):
        configuration = {} if configuration is None else configuration

        self.input_map = inputs.get("Map", None)
        self.output_markers = outputs.get("Markers", None)
        self.output_divisions = outputs.get("Divisions", None)
        self.output_debug_img = outputs.get("DebugImage", None)

        #Add the common configuration to this node's configuration
        common_cfg_file = str(configuration.get("common_cfg_file", "config/common_cfg.yaml"))
        common_cfg_yaml_file = open(common_cfg_file)
        common_cfg_dict = yaml.load(common_cfg_yaml_file, Loader=yaml.FullLoader)
        common_cfg_yaml_file.close()
        configuration.update(common_cfg_dict)

        self.map_parts = int(configuration.get("swarm_size", 2))
        self.robot_namespaces = list(configuration.get("robot_namespaces", ["robot1", "robot2"]))
        self.ns_bytes_lenght = int(configuration.get("ns_bytes_lenght", 64))

        if self.map_parts <= 1:
            raise Exception("ERROR: swarm_size argument should be at least 1.")
        if len(self.robot_namespaces) != self.map_parts:
            raise Exception("ERROR: wrong number of namespaces given.")

        check_for_type_support(OccupancyGrid)
        check_for_type_support(Point32)

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
        factors = self.factorize(self.map_parts)

        if len(factors) == 0: #For only one robot
            return [1, 1]
        elif len(factors) == 1:
            factors.append(1)
            return factors
        elif len(factors) == 2:
            return factors
        else: #More than one single configuration
            return self.get_squarest_distribution(factors)

    def divide_map(self):
        division_shape = self.get_division_shape()
        
        if self.map_msg.info.width > self.map_msg.info.height:
            if division_shape[0] > division_shape[1]:
                division_shape = list(reversed(division_shape))
        else:
            if division_shape[0] < division_shape[1]:
                division_shape = list(reversed(division_shape))
        print(f"MAP_DIVIDER_OP -> Map division: {division_shape}")

        width, height = self.map_reduced_shape
        x_shift = width / division_shape[0]
        y_shift = height / division_shape[1]

        debug_img = np.array(self.map_msg.data)
        debug_img.resize((self.map_msg.info.width, self.map_msg.info.height))

        for i in range(division_shape[0]+1):
            cv2.line(debug_img,
                     [round(self.map_bbox[0].x + (x_shift*i)), round(self.map_bbox[0].y)],
                     [round(self.map_bbox[0].x + (x_shift*i)), round(self.map_bbox[0].y + height)],
                     0, 1)
        for j in range(division_shape[1]+1):
            cv2.line(debug_img,
                     [round(self.map_bbox[0].x), round(self.map_bbox[0].y + y_shift*j)],
                     [round(self.map_bbox[0].x + width), round(self.map_bbox[0].y + y_shift*j)],
                     0, 1)

        bboxes = [] #Map divided into N bboxes:
        for i in range(division_shape[0]):
            for j in range(division_shape[1]):
                bboxes.append([(round(self.map_bbox[0].x + x_shift * i),
                                round(self.map_bbox[0].y + y_shift * j)),
                               (round(self.map_bbox[0].x + x_shift * (i+1)),
                                round(self.map_bbox[0].y + y_shift * (j+1)))])
        #TO DEBUG:
        #print(bboxes)
        #for bbox in bboxes:
        #    for point in bbox:
        #        cv2.circle(debug_img,point, 2, 0, 2, -1)

        #for i in range(division_shape[0]+1):
        #    for j in range(division_shape[1]+1):
        #        cv2.circle(debug_img,
        #                   (round(self.map_bbox[0].x + (x_shift*i)), round(self.map_bbox[0].y + y_shift*j)),
        #                   2, 0, 2, -1)
        
        #cv2.imwrite("/tmp/map_test_divided.png", debug_img) #For debug
        #TODO: publish the debug image in a topic to see it from rviz2.
        #TODO: publish the debug markers in a topic to see them from rviz2.

        return bboxes

    def create_div_msg(self, bbox: list, namespace: str) -> bytes:
        #Namespace is used as an ID, and is put at the beginning:
        ns_bytes = bytes(namespace, "utf-8")
        #Fixed ns lenght of self.ns_bytes_lenght:
        div_msg = bytes(self.ns_bytes_lenght - len(ns_bytes)) + ns_bytes #Fill with null bytes.
        for point in bbox:
            p = Point32()
            p.x, p.y = (float(point[0]), float(point[1]))
            div_msg += _rclpy.rclpy_serialize(p, type(p))
        return div_msg

    async def iteration(self) -> None:
        data_msg = await self.input_map.recv()
        print("MAP_DIVIDER_OP -> map and bbox message received")

        #First 64 bytes correspond to the bbox (32 bytes for each point = 16 hexadecimal units):
        p32_hsize = len(_rclpy.rclpy_serialize(Point32(), Point32)) # 16
        self.map_bbox = (_rclpy.rclpy_deserialize(data_msg.data[:p32_hsize], Point32),
                         _rclpy.rclpy_deserialize(data_msg.data[p32_hsize:p32_hsize*2], Point32))
        self.map_reduced_shape = (self.map_bbox[1].x - self.map_bbox[0].x,
                                  self.map_bbox[1].y - self.map_bbox[0].y) #(width, height)
        self.map_msg = _rclpy.rclpy_deserialize(data_msg.data[p32_hsize*2:], OccupancyGrid)

        for bbox, ns in zip(self.divide_map(), self.robot_namespaces):
            div_msg = self.create_div_msg(bbox, ns)
            await self.output_divisions.send(div_msg)
            print(f"MAP_DIVIDER_OP -> Sending division limits to:\t{ns}\t{bbox}")

        return None

    def finalize(self) -> None:
        return None



def register():
    return MapDivider
