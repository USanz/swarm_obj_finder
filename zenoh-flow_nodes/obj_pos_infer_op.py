from zenoh_flow.interfaces import Operator
from zenoh_flow import Input, Output
from zenoh_flow.types import Context
from typing import Dict, Any

import time, asyncio
from numpy import arctan2, rad2deg, cos, sin, deg2rad

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo, LaserScan
from visualization_msgs.msg import MarkerArray

import sys, os, inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
sys.path.insert(0, currentdir)
from comms_utils import *
from geom_utils import *
from marker_utils import *



class PathsPlanner(Operator):
    def __init__(
        self,
        context: Context,
        configuration: Dict[str, Any],
        inputs: Dict[str, Input],
        outputs: Dict[str, Output],
    ):
        configuration = {} if configuration is None else configuration

        self.input_robot_pose1 = inputs.get("RobotPose1", None)
        self.input_robot_pose2 = inputs.get("RobotPose2", None)
        self.input_obj_detected = inputs.get("ObjDetected", None)

        input_cam_info1 = inputs.get("CamInfo1", None)
        input_cam_info2 = inputs.get("CamInfo2", None)
        self.inputs_cam_info = [input_cam_info1, input_cam_info2]

        self.input_lidar1 = inputs.get("Lidar1", None)
        self.input_lidar2 = inputs.get("Lidar2", None)
        self.inputs_lidar = [self.input_lidar1, self.input_lidar2]

        self.output_world_pos = outputs.get("WorldObjPose", None)
        self.output_debug_marker = outputs.get("DebugMarkers", None)

        #Add the common configuration to this node's configuration
        #common_cfg_file = str(configuration.get("common_cfg_file",
        #                                        "config/common_cfg.yaml"))
        #common_cfg_yaml_file = open(common_cfg_file)
        #common_cfg_dict = yaml.load(common_cfg_yaml_file,
        #                            Loader=yaml.FullLoader)
        #common_cfg_yaml_file.close()
        #configuration.update(common_cfg_dict)

        self.robot_num = int(configuration.get("swarm_size", 2))
        self.robot_namespaces = list(configuration.get("robot_namespaces",
                                                       ["robot1", "robot2"]))
        self.ns_bytes_lenght = int(configuration.get("ns_bytes_lenght", 64))
        self.int_bytes_lenght = int(configuration.get("int_bytes_lenght", 4))
        self.lidar_threshold = int(configuration.get("lidar_threshold", 4))

        self.first_time = True
        self.pending = list()
        self.cam_infos = [CameraInfo()] * self.robot_num
        self.robot_poses = [PoseStamped()] * self.robot_num
        self.lidars = [LaserScan()] * self.robot_num
        self.last_time = time.time()

    def lidar_mean(self, lidar: LaserScan, angle: int, thresh: int) -> float:
        lidar_ht = round(thresh / 2)
        avg = 0
        print("distance:", lidar.ranges[angle])
        counter = 0
        for i in range(angle - lidar_ht, angle + lidar_ht):
            if i >= 360:
                i -= 360
            if lidar.range_min < lidar.ranges[i] < lidar.range_max:
                avg += lidar.ranges[i]
                counter += 1
            
        if avg == 0:
            return None
        avg /= counter
        return avg

    def img2world(self, pix: tuple, cam_info: CameraInfo,
                  rob_pose: PoseStamped, lidar: list) -> tuple:
        #Get the angle of view (AOV) from the cam info:
        film_size_x = cam_info.width
        obj_x_img = pix[0]
        f_x = cam_info.k[0] #Focal point x (from 3x3 k matrix).
        #f_y = cam_info.k[4] #Focal point y (not needed).
        aov_h = 2 * arctan2(film_size_x, 2*f_x) #Horizontal angle of view.

        #Angle of the object from the robot between [-aov_h/2, aov_h/2]:
        obj_ang = - rad2deg((aov_h * obj_x_img / film_size_x) - (aov_h / 2))
        obj_ang = round(obj_ang)

        #Get the dist of the object to get its polar coords from robot's pose:
        #TODO: We can substract like 10cm or so to this distance to make the robots stay further from the obj and avoid collisions with it.
        obj_dist = self.lidar_mean(lidar, obj_ang, self.lidar_threshold)
        roll, pitch, yaw = quat2euler(rob_pose.pose.orientation)
        tot_ang = yaw + deg2rad(obj_ang)
        x_world_dist_from_rob = obj_dist * cos(tot_ang)
        y_world_dist_from_rob = obj_dist * sin(tot_ang)

        world_pose = PoseStamped()
        world_pose.pose.position.x = float(rob_pose.pose.position.x + x_world_dist_from_rob)
        world_pose.pose.position.y = float(rob_pose.pose.position.y + y_world_dist_from_rob)
        world_pose.pose.orientation = rob_pose.pose.orientation
        
        ###DEBUG:
        marker_arr = MarkerArray()
        #Marker (blue) from map frame (absolute coords)
        marker_dict = {"id": 1000, "ns": "obj_pose", "frame_locked": False,
                       "frame_id":"map", "lifetime_s": 0, "lifetime_ns":0,
                       "pose": [world_pose.pose.position.x,
                                world_pose.pose.position.y,
                                world_pose.pose.orientation], # [x, y, yaw(quat)]
                       "scale": [0.2, 0.1, 0.1], "color_rgba": [0.2, 0.2, 1.0, 1.0]}
        marker_arr.markers.append(get_marker(marker_dict))

        #Marker (pinkie) from base_scan frame (relative to the robot coords)
        #marker_dict = {"id": 1001, "ns": "obj_pose", "frame_locked": False,
        #               "frame_id":"base_scan", "lifetime_s": 0, "lifetime_ns":0,
        #               "pose": [obj_dist * cos(deg2rad(obj_ang)),
        #                        obj_dist * sin(deg2rad(obj_ang)),
        #                        Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)], # [x, y, yaw(quat)]
        #               "scale": [0.2, 0.1, 0.1], "color_rgba": [0.7, 0.5, 1.0, 1.0]}
        #marker_arr.markers.append(get_marker(marker_dict))
        ###

        debug_info = [marker_arr, obj_ang, obj_dist,
                      x_world_dist_from_rob, y_world_dist_from_rob]
        return (world_pose, debug_info)

    async def wait_obj_detected(self):
        data_msg = await self.input_obj_detected.recv()
        return ("ObjDetected", data_msg)

    async def wait_robot_pose1(self):
        data_msg = await self.input_robot_pose1.recv()
        return ("RobotPose1", data_msg)

    async def wait_robot_pose2(self):
        data_msg = await self.input_robot_pose2.recv()
        return ("RobotPose2", data_msg)

    async def wait_lidar1(self):
        data_msg = await self.input_lidar1.recv()
        return ("Lidar1", data_msg)

    async def wait_lidar2(self):
        data_msg = await self.input_lidar2.recv()
        return ("Lidar2", data_msg)
    
    def create_task_list(self):
        task_list = [] + self.pending
        if not any(t.get_name() == "ObjDetected" for t in task_list):
            task_list.append(
                asyncio.create_task(self.wait_obj_detected(), name="ObjDetected")
            )
        if not any(t.get_name() == "RobotPose1" for t in task_list):
            task_list.append(
                asyncio.create_task(self.wait_robot_pose1(), name="RobotPose1")
            )
        if not any(t.get_name() == "RobotPose2" for t in task_list):
            task_list.append(
                asyncio.create_task(self.wait_robot_pose2(), name="RobotPose2")
            )
        if not any(t.get_name() == "Lidar1" for t in task_list):
            task_list.append(
                asyncio.create_task(self.wait_lidar1(), name="Lidar1")
            )
        if not any(t.get_name() == "Lidar2" for t in task_list):
            task_list.append(
                asyncio.create_task(self.wait_lidar2(), name="Lidar2")
            )
        return task_list

    async def iteration(self) -> None:

        if self.first_time:
            #Get the cam info of each robot
            for i in range(self.robot_num):
                cam_info_msg_ser = await self.inputs_cam_info[i].recv()
                self.cam_infos[i] = deser_ros2_msg(cam_info_msg_ser.data, CameraInfo)
            self.first_time = False

        (done, pending) = await asyncio.wait(
            self.create_task_list(),
            return_when=asyncio.FIRST_COMPLETED,
        )
        self.pending = list(pending)
        for d in done:
            (who, data_msg) = d.result()

            if "RobotPose" in who: #who contains "RobotPose".
                index = int(who[-1]) -1 #who should be RobotPose1, RobotPose2, ...
                self.robot_poses[index] = deser_ros2_msg(data_msg.data,
                                                         PoseStamped)

            if "Lidar" in who: #who contains "Lidar".
                index = int(who[-1]) -1 #who should be Lidar1, Lidar2, ...
                self.lidars[index] = deser_ros2_msg(data_msg.data, LaserScan)

            if who == "ObjDetected":
                ser_ns = data_msg.data[:self.ns_bytes_lenght]
                ns = deser_string(ser_ns)
                index = self.robot_namespaces.index(ns)
                centroid = deser_int_list(
                    data_msg.data[self.ns_bytes_lenght:], self.int_bytes_lenght
                    )
                world_pose, debug_info = self.img2world(tuple(centroid),
                                                        self.cam_infos[index],
                                                        self.robot_poses[index],
                                                        self.lidars[index])
                if time.time() - self.last_time > 1.0: #send the pose every second
                    debug_marker_msg, ang, dist, xdist, ydist,  = debug_info

                    ser_world_pos = ser_ros2_msg(world_pose)
                    await self.output_world_pos.send(ser_ns + ser_world_pos)

                    #TODO: maybe it's easier to request the transform from map to
                    #base_footprint and save the transform from base_footprint to the object
                    #for then lookup the transform from map to the marker new TF.

                    ser_debug_marker = ser_ros2_msg(debug_marker_msg)
                    await self.output_debug_marker.send(ser_debug_marker)
                    self.last_time = time.time()

                    #DEBUG INFO
                    world_pos = [self.robot_poses[index].pose.position.x,
                                 self.robot_poses[index].pose.position.y]
                    print(f"[{self.last_time}] OBJ_POS_INFER_OP -> Robot world pos: {world_pos}")
                    print(f"[{self.last_time}] OBJ_POS_INFER_OP -> Object angle [º]: {ang}")
                    print(f"[{self.last_time}] OBJ_POS_INFER_OP -> Object dist [m]: {dist} -> [{xdist}, {ydist}]\n")

        return None

    def finalize(self) -> None:
        return None



def register():
    return PathsPlanner
