from zenoh_flow.interfaces import Operator
from zenoh_flow import Input, Output
from zenoh_flow.types import Context
from typing import Dict, Any

import yaml, asyncio, time
from math import sqrt

from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.type_support import check_for_type_support

from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Duration
from tf2_msgs.msg import TFMessage
import tf2_ros, rclpy


import sys, os, inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
sys.path.insert(0, currentdir)
from comms_utils import *
from geom_utils import *



class Navigator(Operator):
    def __init__(
        self,
        context: Context,
        configuration: Dict[str, Any],
        inputs: Dict[str, Input],
        outputs: Dict[str, Output],
    ):
        configuration = {} if configuration is None else configuration

        self.input_next_wp = inputs.get("NextWP", None)
        self.input_world_pos = inputs.get("WorldPosition", None)
        self.input_tf1 = inputs.get("TF1", None)
        self.input_tf2 = inputs.get("TF2", None)

        self.output_wp_req = outputs.get("WPRequest", None)

        self.output_wps1 = outputs.get("Waypoint1", None)
        self.output_wps2 = outputs.get("Waypoint2", None)
        self.wp_outputs = [self.output_wps1, self.output_wps2]

        #Add the common configuration to this node's configuration
        common_cfg_file = str(configuration.get("common_cfg_file", "config/common_cfg.yaml"))
        common_cfg_yaml_file = open(common_cfg_file)
        common_cfg_dict = yaml.load(common_cfg_yaml_file, Loader=yaml.FullLoader)
        common_cfg_yaml_file.close()
        configuration.update(common_cfg_dict)

        self.robot_num = int(configuration.get("swarm_size", 2))
        self.robot_namespaces = list(configuration.get("robot_namespaces", ["robot1", "robot2"]))
        self.ns_bytes_lenght = int(configuration.get("ns_bytes_lenght", 64))

        self.goal_checker_min_dist = float(configuration.get("goal_checker_min_dist", 0.3))
        self.goal_resend_timeout = float(configuration.get("goal_resend_timeout", 1.0))
        self.first_time = True

        self.buffer_core = tf2_ros.BufferCore(Duration(sec=1,
                                                       nanosec=0)) # 0.7s

        self.pending = list()
        self.current_wps = [[PoseStamped(), time.time(), -1.0]] * self.robot_num
        self.object_found = False

        check_for_type_support(PoseStamped)
    
    async def wait_tf1(self):
        data_msg = await self.input_tf1.recv()
        return ("TF1", data_msg)
    
    async def wait_tf2(self):
        data_msg = await self.input_tf2.recv()
        return ("TF2", data_msg)
    
    async def wait_next_wp(self):
        data_msg = await self.input_next_wp.recv()
        return ("NextWP", data_msg)

    async def wait_world_pos(self):
        data_msg = await self.input_world_pos.recv()
        return ("WorldPosition", data_msg)

    def create_task_list(self):
        task_list = [] + self.pending
        if not any(t.get_name() == "TF1" for t in task_list):
            task_list.append(
                asyncio.create_task(self.wait_tf1(), name="TF1")
            )
        if not any(t.get_name() == "TF2" for t in task_list):
            task_list.append(
                asyncio.create_task(self.wait_tf2(), name="TF2")
            )
        if not any(t.get_name() == "NextWP" for t in task_list):
            task_list.append(
                asyncio.create_task(self.wait_next_wp(), name="NextWP")
            )
        if not any(t.get_name() == "WorldPosition" for t in task_list):
            task_list.append(
                asyncio.create_task(self.wait_world_pos(), name="WorldPosition")
            )
        return task_list

    async def iteration(self) -> None:
        #Make the first request for each robot:
        if self.first_time:
            for ns in self.robot_namespaces:
                print(f"NAVIGATOR_OP -> {ns} sending first waypoint request...")
                ns_ser = ser_string(ns, self.ns_bytes_lenght, ' ')
                await self.output_wp_req.send(ns_ser)
            self.first_time = False

        (done, pending) = await asyncio.wait(
            self.create_task_list(),
            return_when=asyncio.FIRST_COMPLETED,
        )
        self.pending = list(pending)
        for d in done:
            (who, data_msg) = d.result()

            if who == "NextWP" and not self.object_found:
                ns = deser_string(data_msg.data[:self.ns_bytes_lenght], ' ')
                index = self.robot_namespaces.index(ns)
                
                ser_current_wp = data_msg.data[self.ns_bytes_lenght:]
                self.current_wps[index] = [deser_ros2_msg(ser_current_wp, PoseStamped),
                                           time.time(), -1.0]
                print(f"NAVIGATOR_OP -> {self.robot_namespaces[index]} received next waypoint, sending it: {get_xy_from_pose(self.current_wps[index][0])}")
                await self.wp_outputs[index].send(ser_current_wp)

            if "TF" in who and not self.object_found: #who contains "TF" or "TF_static".
                index = int(who[-1]) -1 #who should be TF1, TF2, ...
                ns = self.robot_namespaces[index]
                self.tf_msg = deser_ros2_msg(data_msg.data, TFMessage)
                for tf in self.tf_msg.transforms:

                    tf_names = ["map->odom",
                                "odom->base_footprint"]
                    key = tf.header.frame_id + "->" + tf.child_frame_id
                    if key in tf_names:
                        self.buffer_core.set_transform(tf, "default_authority")
                    
                        try:
                            new_tf = self.buffer_core.lookup_transform_core('map', 'base_footprint', rclpy.time.Time())
                            new_tf.child_frame_id = 'world_robot_pos'

                            x_dist = new_tf.transform.translation.x - self.current_wps[index][0].pose.position.x
                            y_dist = new_tf.transform.translation.y - self.current_wps[index][0].pose.position.y
                            dist = sqrt(x_dist**2 + y_dist**2)
                            #print(ns, "distance:", dist)
                            self.current_wps[index][2] = dist
                            #ori_err = quat_diff(pose_stamped.pose.pose.orientation - self.current_wps[index][0].pose.orientation)
                            if dist < self.goal_checker_min_dist:# and ori_err < radians(5):
                                print("NAVIGATOR_OP -> Waypoint reached, sending next request...")
                                ns = self.robot_namespaces[index]
                                ns_ser = ser_string(ns, self.ns_bytes_lenght, ' ')
                                await self.output_wp_req.send(ns_ser)
                            
                        except Exception as e:
                            pass
                            #print(e)
            
            if who == "WorldPosition":
                self.object_found = True
                ser_ns = data_msg.data[:self.ns_bytes_lenght]
                ns = deser_string(ser_ns)
                index = self.robot_namespaces.index(ns)
                ser_msg = ser_ros2_msg(self.current_wps[index][0])
                for i, output in enumerate(self.wp_outputs):
                    print(f"NAVIGATOR_OP -> Sending {self.robot_namespaces[i]} to {ns}'s position: {self.current_wps[index][0]}")
                    output.send(ser_msg) # Send them all to the position of the robot who found the object
    
    def finalize(self) -> None:
        return None



def register():
    return Navigator
