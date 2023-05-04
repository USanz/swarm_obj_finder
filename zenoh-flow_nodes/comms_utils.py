from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.type_support import check_for_type_support



def ser_string(s: str, s_lenght: int, fill_char=' ') -> bytes:
    ns_bytes = bytes(s, "utf-8")
    #Fixed ns lenght of s_lenght (fill with space characters):
    return bytes(fill_char, "utf-8") * (s_lenght - len(ns_bytes)) + ns_bytes

def deser_string(bytes: bytes, fill_chars=' ') -> str:
    return str(bytes.decode('utf-8')).lstrip(fill_chars)

def ser_ros2_msg(ros2_msg) -> bytes:
    check_for_type_support(type(ros2_msg))
    return _rclpy.rclpy_serialize(ros2_msg, type(ros2_msg))

def deser_ros2_msg(ser_ros2_msg: bytes, ros2_type):
    check_for_type_support(ros2_type)
    return _rclpy.rclpy_deserialize(ser_ros2_msg, ros2_type)
