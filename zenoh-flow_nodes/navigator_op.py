from zenoh_flow.interfaces import Operator
from zenoh_flow import Input, Output
from zenoh_flow.types import Context
from typing import Dict, Any

import yaml, asyncio

from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.type_support import check_for_type_support

#from nav2_msgs.action import NavigateToPose
#from nav2_msgs.action._navigate_to_pose import NavigateToPose_Feedback as Feedback
#topic: "/robot1/navigate_to_pose/_action/feedback", this works:
#from nav2_msgs.action._navigate_to_pose import NavigateToPose_FeedbackMessage as FeedbackMessage
#topic: "/robot1/navigate_to_pose/_action/status":
from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import Point32, PoseStamped, Pose



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
        self.input_nav_status1 = inputs.get("NavStatus1", None)
        self.input_nav_status2 = inputs.get("NavStatus2", None)

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

        self.pending = list()
        self.paths = [[]] * self.robot_num
        self.current_goals = [PoseStamped()] * self.robot_num

        check_for_type_support(Point32)
        check_for_type_support(PoseStamped)
        check_for_type_support(Pose)
        check_for_type_support(GoalStatusArray)


    def stamp_pose(self, pose: Pose) -> PoseStamped: #Pose to PoseStamped
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.header.stamp.sec = 0
        goal_pose.header.stamp.nanosec = 0
        goal_pose.pose = pose
        return goal_pose

    def ser_goal_pose_msg(self, goal_pose_msg: PoseStamped) -> bytes:
        return _rclpy.rclpy_serialize(goal_pose_msg, PoseStamped)
    
    async def wait_paths(self):
        data_msg = await self.input_paths.recv()
        return ("Paths", data_msg)
    
    async def wait_status1(self):
        data_msg = await self.input_nav_status1.recv()
        return ("NavStatus1", data_msg)
    
    async def wait_status2(self):
        data_msg = await self.input_nav_status2.recv()
        return ("NavStatus2", data_msg)

    def create_task_list(self):
        task_list = [] + self.pending

        if not any(t.get_name() == "Paths" for t in task_list):
            task_list.append(
                asyncio.create_task(self.wait_paths(), name="Paths")
            )
        if not any(t.get_name() == "NavStatus1" for t in task_list):
            task_list.append(
                asyncio.create_task(self.wait_status1(), name="NavStatus1")
            )
        if not any(t.get_name() == "NavStatus2" for t in task_list):
            task_list.append(
                asyncio.create_task(self.wait_status2(), name="NavStatus2")
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
            if who == "Paths":
                ns_bytes = data_msg.data[:self.ns_bytes_lenght]
                path_len_bytes = data_msg.data[self.ns_bytes_lenght:self.ns_bytes_lenght + self.int_bytes_lenght]
                path_bytes = data_msg.data[self.ns_bytes_lenght + self.int_bytes_lenght:]
                print("NAVIGATOR_OP -> path received")

                #TODO: create robotPath objects for every namespace to store their paths:
                path_len = int.from_bytes(path_len_bytes, 'little')

                ns_from_bytes = str(ns_bytes.decode('utf-8')).lstrip()
                #We need to get the original name because in the decoded there
                #are empty bytes (/x00) that can't be removed otherwise:
                index = self.robot_namespaces.index(ns_from_bytes)
                ns = self.robot_namespaces[index]

                path = []
                bytes_step = len(_rclpy.rclpy_serialize(Pose(), Pose))
                for starting_byte in range(0, path_len*bytes_step, bytes_step):
                    path.append(_rclpy.rclpy_deserialize(path_bytes[starting_byte:starting_byte + bytes_step], Pose))
                self.paths[index] = path

                first_wp = self.paths[index].pop(0)
                goal_pose_msg = self.stamp_pose(first_wp) #Pose to PoseStamped
                goal_pose_ser = self.ser_goal_pose_msg(goal_pose_msg)
                self.wp_outputs[index].send(goal_pose_ser)
                print("NAVIGATOR_OP -> sending first goal pose",
                      goal_pose_msg.pose.position, "to", ns)
                
            if "NavStatus" in who: #who contains "NavStatus"
                index = int(who[-1]) -1 #who should be NavStatus1, NavStatus2, ....
                status_msg = _rclpy.rclpy_deserialize(data_msg.data, GoalStatusArray)
                #This also works: most of the times gets a status of 4 when
                #reached but sometimes it gets a 6 in the middle and doesn't
                #receive the 4 afterwards when reachnig the goal.
                
                status = status_msg.status_list[-1].status
                print("NAVIGATOR_OP ->", who, "status received:", status)
                if status == 4: #When goal reached, send the next one:
                    if len(self.paths[index]) > 0:
                        next_wp = self.paths[index].pop(0)
                        self.current_goals[index] = self.stamp_pose(next_wp)
                        goal_pose_ser = self.ser_goal_pose_msg(self.current_goals[index])
                        self.wp_outputs[index].send(goal_pose_ser)
                        print("NAVIGATOR_OP ->", who,
                              "sending next wp:", self.current_goals[index])
                    else:
                        print("NAVIGATOR_OP ->", who, "path finished!")
                elif status == 6: #In case of navigation failure, resend the goal:
                    print("NAVIGATOR_OP ->", who,
                          "resending wp:", self.current_goals[index])
                    goal_pose_ser = self.ser_goal_pose_msg(self.current_goals[index])
                    self.wp_outputs[index].send(goal_pose_ser)

    def finalize(self) -> None:
        return None



def register():
    return Navigator
