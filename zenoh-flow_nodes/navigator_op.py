from zenoh_flow.interfaces import Operator
from zenoh_flow import Input, Output
from zenoh_flow.types import Context
from typing import Dict, Any

import yaml, asyncio
from math import sqrt

from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.type_support import check_for_type_support

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

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
        self.input_pose1 = inputs.get("Pose1", None)
        self.input_pose2 = inputs.get("Pose2", None)

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
        self.int_bytes_lenght = int(configuration.get("int_bytes_lenght", 4))

        #self.resend_timeout_ms = int(configuration.get("resend_timeout_ms", 1000))
        self.first_time = True

        self.pending = list()
        self.current_wps = [PoseStamped()] * self.robot_num

        check_for_type_support(PoseStamped)
        check_for_type_support(PoseWithCovarianceStamped)
    
    async def wait_pose1(self):
        data_msg = await self.input_pose1.recv()
        return ("Pose1", data_msg)
    
    async def wait_pose2(self):
        data_msg = await self.input_pose2.recv()
        return ("Pose2", data_msg)
    
    async def wait_next_wp(self):
        data_msg = await self.input_next_wp.recv()
        return ("NextWP", data_msg)

    def create_task_list(self):
        task_list = [] + self.pending

        if not any(t.get_name() == "Pose1" for t in task_list):
            task_list.append(
                asyncio.create_task(self.wait_pose1(), name="Pose1")
            )
        if not any(t.get_name() == "Pose2" for t in task_list):
            task_list.append(
                asyncio.create_task(self.wait_pose2(), name="Pose2")
            )
        if not any(t.get_name() == "NextWP" for t in task_list):
            task_list.append(
                asyncio.create_task(self.wait_next_wp(), name="NextWP")
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

            if who == "NextWP":
                ns = deser_string(data_msg.data[:self.ns_bytes_lenght], ' ')
                index = self.robot_namespaces.index(ns)
                
                ser_current_wp = data_msg.data[self.ns_bytes_lenght:]
                self.current_wps[index] = deser_ros2_msg(ser_current_wp, PoseStamped)
                print(f"NAVIGATOR_OP -> {self.robot_namespaces[index]} received next waypoint: {get_xy_from_pose(self.current_wps[index])}")
                await self.wp_outputs[index].send(ser_current_wp)

            #Get the poses from odom and transform to map
            if "Pose" in who: #who contains "Pose"
                index = int(who[-1]) -1 #who should be Pose1, Pose2, ...
                pose_stamped = deser_ros2_msg(data_msg.data, PoseWithCovarianceStamped)
                x_dist = pose_stamped.pose.pose.position.x - self.current_wps[index].pose.position.x
                y_dist = pose_stamped.pose.pose.position.y - self.current_wps[index].pose.position.y
                #ori_err = quat_diff(pose_stamped.pose.pose.orientation - self.current_wps[index].pose.orientation)
                dist = sqrt(x_dist**2 + y_dist**2)
                print(f"NAVIGATOR_OP -> Distance from {self.robot_namespaces[index]} to its next wp: {dist}")

                if dist < 0.4:# and ori_err < radians(5):
                    print("NAVIGATOR_OP -> Waypoint reached, sending next request...")
                    ns = self.robot_namespaces[index]
                    ns_ser = ser_string(ns, self.ns_bytes_lenght, ' ')
                    await self.output_wp_req.send(ns_ser)

    def finalize(self) -> None:
        return None



def register():
    return Navigator
