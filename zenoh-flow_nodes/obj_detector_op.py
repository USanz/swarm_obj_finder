from zenoh_flow.interfaces import Operator
from zenoh_flow import Input, Output
from zenoh_flow.types import Context
from typing import Dict, Any

import asyncio, numpy as np, cv2

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import sys, os, inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
sys.path.insert(0, currentdir)
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

        self.input_img1 = inputs.get("Image1", None)
        self.input_img2 = inputs.get("Image2", None)
        
        self.output_obj_detected = outputs.get("ObjDetected", None)
        output_debug_img1 = outputs.get("DebugImgFiltered1", None)
        output_debug_img2 = outputs.get("DebugImgFiltered2", None)
        self.output_debug_imgs = [output_debug_img1, output_debug_img2]

        #Add the common configuration to this node's configuration
        #common_cfg_file = str(configuration.get("common_cfg_file",
        #                                        "config/common_cfg.yaml"))
        #common_cfg_yaml_file = open(common_cfg_file)
        #common_cfg_dict = yaml.load(common_cfg_yaml_file,
        #                            Loader=yaml.FullLoader)
        #common_cfg_yaml_file.close()
        #configuration.update(common_cfg_dict)

        self.robot_namespaces = list(configuration.get("robot_namespaces",
                                                       ["robot1", "robot2"]))
        self.ns_bytes_lenght = int(configuration.get("ns_bytes_lenght", 64))     
        self.int_bytes_lenght = int(configuration.get("int_bytes_lenght", 4))

        self.lower_threshold = np.array(list(configuration.get("lower_color_filter_threshold",
                                                       [0, 195, 75])))
        self.upper_threshold = np.array(list(configuration.get("upper_color_filter_threshold",
                                                       [16, 225, 105])))
        
        self.bridge = CvBridge()
        self.obj_found = False

        self.pending = list()

    def detect_object(self, img: np.ndarray, x_pix_step: int, y_pix_step: int,
                      lower_threshold: np.array, upper_threshold: np.array) -> tuple:
        height, width, _ = img.shape # 1920 x 1080
        hsv_img = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)

        #mask = cv2.inRange(hsv_img, lower_threshold, upper_threshold)
        #result = cv2.bitwise_and(img, img, mask = mask)
        
        centroid = [0, 0]
        num = 0
        for i in range(0, width, x_pix_step):
            for j in range(0, height, y_pix_step):
                if ((lower_threshold < hsv_img[j, i]).all() and
                    (hsv_img[j, i] < upper_threshold).all()):
                    centroid[0] += i
                    centroid[1] += j
                    num += 1
                    cv2.circle(img, (i, j), 10, (255, 255, 0), -1) #DEBUG

        if num != 0:
            centroid[0] /= num
            centroid[1] /= num
        else:
            centroid = None

        return (centroid, ser_ros2_msg(self.bridge.cv2_to_imgmsg(img)))

    async def wait_img1(self):
        data_msg = await self.input_img1.recv()
        return ("Image1", data_msg)
    
    async def wait_img2(self):
        data_msg = await self.input_img2.recv()
        return ("Image2", data_msg)
    
    def create_task_list(self):
        task_list = [] + self.pending
        if not any(t.get_name() == "Image1" for t in task_list):
            task_list.append(
                asyncio.create_task(self.wait_img1(), name="Image1")
            )
        if not any(t.get_name() == "Image2" for t in task_list):
            task_list.append(
                asyncio.create_task(self.wait_img2(), name="Image2")
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

            if "Image" in who: #who contains "Image".
                index = int(who[-1]) -1 #who should be Image1, Image2, ...
                img_msg = deser_ros2_msg(data_msg.data, Image)
                img = self.bridge.imgmsg_to_cv2(img_msg,
                                                desired_encoding='passthrough')

                centroid, ser_debug_img = self.detect_object(img, 100, 100,
                                                         self.lower_threshold,
                                                         self.upper_threshold)
                await self.output_debug_imgs[index].send(ser_debug_img)

                if centroid != None:# and not self.obj_found:
                    ser_msg = ser_string(self.robot_namespaces[index],
                                         self.ns_bytes_lenght)
                    ser_msg += ser_int_list(centroid,
                                            self.int_bytes_lenght)
                    await self.output_obj_detected.send(ser_msg)
                    self.obj_found = True
        
        return None

    def finalize(self) -> None:
        return None



def register():
    return PathsPlanner
