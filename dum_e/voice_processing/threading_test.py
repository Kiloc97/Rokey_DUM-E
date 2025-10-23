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

rclpy.init()
dsr_node = rclpy.create_node("robot_control_node", namespace=ROBOT_ID)
DR_init.__dsr__node = dsr_node

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
        self.get_keyword_srv = self.create_service(
            SetBool, "get_keyword", self.get_keyword
        )

        self.wake_server = self.create_service(
            SetBool, 'wakeup_teleop', self.wakeup_callback
        )

        self.move_stop_client = self.create_client(MoveStop, 'dsr01/motion/move_stop')
        self.move_resume_client = self.create_client(MoveResume, 'dsr01/motion/move_resume')
        
        while not self.move_stop_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Wait stope move server')

        self.teleop_event = threading.Event()
        threading.Thread(target=self.play, daemon=True).start()
        
        self.wakeup = False
        self.keyword = False

        # movej(posj([0, 0, 90, 0, 90, 0]), vel=100, acc=100)

        # amovel(posx([200, 0, 0, 0, 0, 0]), vel=30, acc=30, mod=DR_MV_MOD_REL)
        # time.sleep(2)
        # self.get_logger().warn('stop')
        # self.move_stop_func(1)
        # time.sleep(5)
        # self.get_logger().warn('restart')
        # self.move_resume_func()

        self.get_logger().warn(f'{get_robot_mode()}')
        self.get_logger().warn(f'{get_tcp()}')

        set_robot_mode(1)
        set_tcp('realsence_tcp')
        time.sleep(2)
        self.get_logger().warn(f'{get_robot_mode()}')
        self.get_logger().warn(f'{get_tcp()}')

    # def move_stop_func(self, num):
    #     move_stop_request = MoveStop()
    #     move_stop_request.stop_mode=num
    #     self.get_logger().info(f"Call move stop service with stop_mode={num}")
    #     move_stop_future=self.move_stop_client.call_async(
    #         move_stop_request
    #     )
    #     # rclpy.spin_until_future_complete(self, move_stop_future)
    #     move_stop_future.add_done_callback(self.stop_callback)
    
    # def stop_callback(self, future):
    #     response = future.result()
    #     if response:
    #         self.get_logger().info('MoveStop service call success')
    #     else:
    #         self.get_logger().info('MoveStop service call failed')

    # def move_resume_func(self):
    #     while not self.move_resume_client.wait_for_service(timeout_sec=1.0):
    #         self.get_logger().info('Wait resume move server')
    #     move_resume_request = MoveStop()
    #     self.get_logger().info(f"Call move resume service")
    #     move_resume_future=self.move_resume_client.call_async(
    #         move_resume_request
    #     )
    #     # rclpy.spin_until_future_complete(self, move_resume_future)
    #     move_resume_future.add_done_callback(self.resume_callback)

    # def resume_callback(self, future):
    #     response = future.result()
    #     if response:
    #         self.get_logger().info('MoveStop service call success')
    #     else:
    #         self.get_logger().info('MoveStop service call failed')

    # def play(self):
    #     self.get_logger().warn('ok')
    #     while not self.wakeup and not self.keyword:
    #         self.get_logger().info('ok')
    #         time.sleep(1)
    #         pass
    #     self.get_logger().warn('end')

    # def get_keyword(self, request, response):  # 요청과 응답 객체를 받아야 함
    #     if request.data:
    #         response.success = True
    #         response.message = 'Recoding Start...'
    #         self.keyword = True
    #     else:
    #         response.success = False
    #         response.message = 'Incorrect key!'
    #         self.keyword = False
        
    #     return response

    # def wakeup_callback(self, request, response):
    #     if request.data:
    #         response.success = True
    #         response.message = 'Recoding Start...'
    #         self.wakeup = True
    #     else:
    #         response.success = False
    #         response.message = 'Incorrect key!'
    #         self.wakeup = False

    #     return response


def main():
    # rclpy.init()
    node = GetKeyword()
    excutor = MultiThreadedExecutor()
    excutor.add_node(node)
    try:
        while rclpy.ok():
            excutor.spin_once()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
