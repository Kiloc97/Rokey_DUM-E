# ros2 service call /get_keyword std_srvs/srv/Trigger "{}"

import rclpy
import threading
from rclpy.node import Node

from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import SetBool
import threading
import time

import DR_init
import sys

from dsr_msgs2.srv import MoveStop
from dsr_msgs2.srv import MoveResume

# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 60, 60
# BUCKET_POS = [445.5, -242.6, 174.4, 156.4, 160.0, -112.5]
GRIPPER_NAME = "rg2"
TOOLCHARGER_IP = "192.168.1.1"
TOOLCHARGER_PORT = "502"
DEPTH_OFFSET = -5.0
MIN_DEPTH = 2.0

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

# rclpy.init()
# dsr_node = rclpy.create_node("robot_control_node", namespace=ROBOT_ID)
# DR_init.__dsr__node = dsr_node

try:
    from DSR_ROBOT2 import (
        movej, movel, get_current_posx, move_periodic, get_current_posj,
        mwait, trans, amove_periodic, amovel, amovec, amoveb, amovesx,
        set_robot_mode, get_tcp, set_tcp, set_tool, get_robot_mode, 
        task_compliance_ctrl, release_compliance_ctrl,
        set_desired_force, check_force_condition, release_force, get_tool_force, 
        DR_MV_MOD_REL, DR_FC_MOD_REL, DR_BASE, DR_TOOL,
        DR_AXIS_X, DR_AXIS_Y, DR_AXIS_Z,
        drl_script_resume, drl_script_stop
    )

    from DR_common2 import posx, posj

except ImportError as e:
    print(f"Error importing DSR_ROBOT2: {e}")
    sys.exit()

############ GetKeyword Node ############
class GetKeyword(Node):
    def __init__(self):
        super().__init__("threading_test_node")

        self.get_logger().warn(f'{get_robot_mode()}')
        self.get_logger().warn(f'{get_tcp()}')

        set_robot_mode(1)
        set_tcp('realsence_tcp')
        time.sleep(2)
        self.get_logger().warn(f'{get_robot_mode()}')
        self.get_logger().warn(f'{get_tcp()}')


def main(args=None):
    # node = GetKeyword(dsr_node)
    # # while rclpy.ok():
    # #     node.robot_control()
    rclpy.shutdown()
    # node.destroy_node()


if __name__ == "__main__":
    main()
