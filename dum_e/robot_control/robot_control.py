import os
import time
import sys
from scipy.spatial.transform import Rotation
import numpy as np
import rclpy
from rclpy.node import Node
import DR_init

from od_msg.srv import SrvDepthPosition
from std_srvs.srv import Trigger
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory
from robot_control.onrobot import RG

from std_srvs.srv import SetBool
from dsr_msgs2.srv import SetRobotSpeedMode
from dsr_msgs2.srv import GetCurrentTcp
import math
from dsr_msgs2.srv import MoveStop

from rclpy.action import ActionClient
from dum_e_interface.action import ActionDetection
from dum_e_interface.srv import Command


package_path = get_package_share_directory("dum_e")

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
        movej, movel, get_current_posx,
        mwait, amovesx, movesj,
        set_robot_mode, set_tcp, 
        task_compliance_ctrl, release_compliance_ctrl,
        set_desired_force, check_force_condition, release_force, 
        DR_MV_MOD_REL, DR_FC_MOD_REL, DR_BASE, DR_TOOL,
        DR_AXIS_X, DR_AXIS_Y, DR_AXIS_Z
    )

    from DR_common2 import posx, posj

except ImportError as e:
    print(f"Error importing DSR_ROBOT2: {e}")
    sys.exit()

########### Gripper Setup. Do not modify this area ############

gripper = RG(GRIPPER_NAME, TOOLCHARGER_IP, TOOLCHARGER_PORT)

########### pos dic ############

workspace_dic = {"tool": [63.21, 15.56, 61.48, 0.05, 103.35, 63.43],
                 "work": [0, 0, 90, 0, 90, 0],
                 "bring": [175.31, 6.68, 293.24, 6.92, -179.73, 6.80]}

tool_dic = {'hammer': [[303.51, 605.03, 196]],
           'monkey': [[42.78, 625.21, 196]],
           '6wrench': [[102.47, 419.22, 196]],
           'screwdriver': [[184.11, 570.52, 196]],
           'pipewrench': [[-10.16, 476.35, 196]],
           'wrench': [[210, 448.77, 196]],
           'knief': [[349.06, 427.79, 196]],
           'voltcup': [[322.26, 652.55, 276]],
           'nutcup': [[140.43, 687.5, 276]]} 

########### Robot Controller ############


class RobotController(Node):
    def __init__(self):
        super().__init__("robot_control_node")
        self.init_robot()

        self.get_position_client = self.create_client(
            SrvDepthPosition, "/get_3d_position"
        )
        while not self.get_position_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().info("Waiting for get_depth_position service...")
        self.get_position_request = SrvDepthPosition.Request()

        self.change_model_client = self.create_client(
            SetBool, "/change_yolo_model"
        )
        
        self.get_keyword_client = self.create_client(Command, "get_keyword")
        self.get_keyword_request = Command.Request()

        self.move_stop_client=self.create_client(
            MoveStop,"/dsr01/motion/move_stop"
        )
        while not self.move_stop_client.wait_for_service(timeout_sec=0.3):
            self.get_logger().info("Waiting for move_stop_client service...")
        self.move_stop_request = MoveStop.Request()

        # self._action_client = ActionClient(self, ActionDetection, 'action_detection')
        self._detection_client = self.create_client(SrvDepthPosition, 'service_detection')
        while not self._detection_client.wait_for_service(timeout_sec=0.3):
            self.get_logger().info("Waiting for _detection_client service...")
        self._detection_request = SrvDepthPosition.Request()

        set_robot_mode(0)
        set_tcp('')


    def robot_control(self):
        self.get_logger().info("call get_keyword service")
        self.get_logger().info("say 'Hello Rokey' and speak what you want to pick up")
        get_keyword_future = self.get_keyword_client.call_async(self.get_keyword_request)
        rclpy.spin_until_future_complete(self, get_keyword_future)        

        if not get_keyword_future.result().success:
            self.get_logger().error(f'get keyword fail')
            return
        try:
            task = get_keyword_future.result().task
            object = get_keyword_future.result().object
            self.get_logger().warn(f'task: {task}')
            self.get_logger().warn(f'object: {object}')
        except ValueError:
            self.get_logger().error(f"Invalid format: {task}, {object}")
            return
        
        self.get_logger().info(f"Task: {task}, Objects: {object}")        
        if task == 'bring':
            self.bring(object)
        elif task == 'clear':
            self.clear([*tool_dic.keys()])
        elif task == 'hold':
            self.hold()
        elif task == 'unhold':
            self.unhold()
        else:
            self.get_logger().error(f'{task}')

# TODO: 
    def change_yolo_model(self, setbool):
        while not self.change_model_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().info("Waiting for change_yolo_model service...")

        change_model_request = SetBool.Request()
        change_model_request.data = setbool
        future = self.change_model_client.call_async(change_model_request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() and future.result().success:
            self.get_logger().info(f"YOLO model changed: {future.result().message}")
        else:
            self.get_logger().warn(f"Failed to change YOLO model: {future.result().message if future.result() else 'No response'}")

    def force_detect(self, force=0, min_force=10):
        self.get_logger().info('task_compliance_ctrl')
        task_compliance_ctrl(stx=[100, 100, 100, 200, 200, 200])
        time.sleep(0.1)
        set_desired_force(fd=[0, 0, force, 0, 0, 0], dir=[1, 1, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
        while check_force_condition(DR_AXIS_Z, min=min_force, ref=DR_BASE) == -1:
            continue
        release_force()
        release_compliance_ctrl()

    def target_detection(self, target):
        self.change_yolo_model(False)
        target_pos = self.get_target_pos(target.strip())
        target_pos[2] = 10
        if target_pos is None:
            self.get_logger().warn("No target position")
        else:
            self.get_logger().info(f"target position: {target_pos}")
        return target_pos
    
    def get_target_pos(self, target):
        self.get_position_request.target = target
        self.get_logger().info("call depth position service with object_detection node")
        get_position_future = self.get_position_client.call_async(
            self.get_position_request
        )
        rclpy.spin_until_future_complete(self, get_position_future)

        if get_position_future.result():
            result = get_position_future.result().depth_position.tolist()
            self.get_logger().info(f"Received depth position: {result}")
            if sum(result) == 0:
                print("No target position")
                return None

            target_pos = self.transform_to_base(result)

        return target_pos
    
    def transform_to_base(self, result):
        gripper2cam_path = os.path.join(
            package_path, "resource", "T_gripper2camera.npy"
        )

        robot_posx = get_current_posx()[0]

        """
        Converts 3D coordinates from the camera coordinate system
        to the robot's base coordinate system.
        """
        gripper2cam = np.load(gripper2cam_path)
        coord = np.append(np.array(result), 1)  # Homogeneous coordinate

        x, y, z, rx, ry, rz = robot_posx
        base2gripper = self.get_robot_pose_matrix(x, y, z, rx, ry, rz)

        # 좌표 변환 (그리퍼 → 베이스)
        base2cam = base2gripper @ gripper2cam
        td_coord = np.dot(base2cam, coord)

        td_coord = td_coord[:3]

        if td_coord[2] and sum(td_coord) != 0:
            td_coord[2] += DEPTH_OFFSET  # DEPTH_OFFSET
            td_coord[2] = max(td_coord[2], MIN_DEPTH)  # MIN_DEPTH: float = 2.0

        target_pos = list(td_coord[:3]) + robot_posx[3:]
        
        return target_pos

    def get_robot_pose_matrix(self, x, y, z, rx, ry, rz):
        R = Rotation.from_euler("ZYZ", [rx, ry, rz], degrees=True).as_matrix()
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = [x, y, z]
        return T

########### Motion ###########
    def open(self):
        gripper.open_gripper()
        while gripper.get_status()[0]:
            time.sleep(0.5)
        time.sleep(0.5)
    
    def close(self):
        gripper.close_gripper()
        while gripper.get_status()[0]:
            time.sleep(0.5)
        time.sleep(0.5)

    def init_robot(self):
            JReady = [0, 0, 90, 0, 90, 0]
            movej(JReady, vel=VELOCITY, acc=ACC)
            self.open()
    
    def move_stop_func(self, num):
        self.move_stop_request.stop_mode=num
        self.get_logger().info(f"Call move stop service with stop_mode={num}")
        move_stop_future=self.move_stop_client.call_async(
            self.move_stop_request
        )
        rclpy.spin_until_future_complete(self,move_stop_future)
        if move_stop_future.done():
            try:
                response=move_stop_future.result()
                if response.success:
                    self.get_logger().info("Move stop service call succeeded.")
                else:
                    self.get_logger().warn("Move stop service responded with failure.")
            except Exception as e:
                self.get_logger().error("Exception during service call: {e}")
        else:
            self.get_logger().error("Move stop service call did not complete.")

    def direciotn(self, target):
        self._detection_request.target = target
        self.get_logger().info("call depth position service with object_detection node")
        get_direction = self._detection_client.call_async(
            self._detection_request
        )
        rclpy.spin_until_future_complete(self, get_direction)

        if get_direction.result():
            result = get_direction.result().depth_position.tolist()
            self.get_logger().info(f"Received depth position: {result}")
            if sum(result) == 0:
                print("No target position")
                return None

        self.move_stop_func(1)
        set_tcp('')
        time.sleep(2)

        target_pos = self.transform_to_base(result)

        return target_pos
    
    def pick(self, target_pos, tool, z):
        self.open()

        target_pos[2] += 150
        set_tcp('realsence_tcp')
        movel(posx(target_pos), vel=VELOCITY, acc=ACC)
        mwait()
        self.change_yolo_model(True)

        self.get_logger().warn(f'{get_current_posx()}')

        amovesx([posx([0, 0, 0, 0, 0, 179]), posx([0, 0, 0, 0, 0, 179])
                , posx([0, 0, 0, 0, 0, -179]), posx([0, 0, 0, 0, 0, -179]),
                posx([0, 0, 0, 0, 0, 179]), posx([0, 0, 0, 0, 0, 179])
                , posx([0, 0, 0, 0, 0, -179]), posx([0, 0, 0, 0, 0, -179])], 
                vel=10, acc=0, time=75, mod=DR_MV_MOD_REL)
        target_pos = self.direciotn(tool)
        
        

        target_pos[2] = z
        movel(target_pos, vel=VELOCITY, acc=ACC)
        mwait()

        self.close()

        movel(posx([0, 0, 100, 0, 0, 0]), vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_REL)
        mwait()
        self.change_yolo_model(False)
    
    # TODO: 동작확인 필요
    def place(self, place_to_pos):
        place_to_pos[2] += 10 # 높이 설정        #place_to_pos에서 받은 좌표로 이동
        self.get_logger().info(f'place_to_pos: {place_to_pos}')
        movel(posx(place_to_pos), vel=VELOCITY, acc=ACC)
        mwait()    

        self.force_detect(force=-10, min_force=6)
        
        movel([0, 0, 10, 0, 0, 0], vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_REL)
        mwait()  
        
        self.open()

########### Task Motion ###########
    def bring(self, get_keyword_result):
        tools = get_keyword_result        

        base_drop_pos = [175.31, 6.68, 293.24, 6.92, -179.73, 6.80]  # 기준 위치
        offset_x = 100  # x 방향 오프셋 (mm 단위)
        for i, tool in enumerate(tools):
            movej(posj(workspace_dic["tool"]), vel=VELOCITY, acc=ACC)
            mwait()            
            for attempt in range(2):
                target_pos = self.get_target_pos(tool.strip())
                if target_pos is None:
                    movel([0, 100, 0, 0, 0, 0], vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_REL)
                    mwait()
                else:
                    self.move_stop_func(1)
                    break            

            if target_pos is None:
                self.get_logger().error(f"Cannot find tool: {tool}!")
                continue  # 다음 tool로 넘어감
            

            self.pick(target_pos, tool, 196)

            movesj([posj(workspace_dic["tool"]), posj(workspace_dic["work"])], vel=VELOCITY, acc=ACC)
            mwait()

            # drop_pos = base_drop_pos.copy()
            base_drop_pos[0] += offset_x * i    

            # if tool in ["nutcup", "voltcup"]:
            #     approach_pos = base_drop_pos.copy()
            #     approach_pos[2] += 50
            #     self.place(approach_pos)
            # else:
            self.place(base_drop_pos)
    
            self.init_robot()
        self.init_robot()
        
    def clear(self, get_keyword_result=[*tool_dic.keys()]):
        tools = get_keyword_result
        task = []
        
        for _ in range(2):
            self.change_yolo_model(False)
            
            movel(posx([0, 0, 100, 0, 0, 0]), vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_REL)
            mwait()
            
            current_pos = get_current_posx()[0]
            while tools:
                self.get_logger().error(f'{tools}')
                tool = tools.pop()
                for pos in tool_dic[tool]:
                    target_pos = self.get_target_pos(tool)
                    if target_pos is None:
                        self.get_logger().warn(f'{tool} no detect')
                        task.append(tool)
                        continue

                    self.pick(target_pos, tool, 290)
                    
                    movesj([posj(workspace_dic["work"]), posj(workspace_dic["tool"])], vel=VELOCITY, acc=ACC)
                    mwait()

                    pos = pos + get_current_posx()[0][3:]
                    self.place(pos)

                    movesj([posj(workspace_dic["tool"]), posj(workspace_dic["work"])], vel=VELOCITY, acc=ACC)
                    mwait()
            
            tools = [*task]

            movel(posx([0, -150, 0, 0, 0, 0]), vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_REL)
            mwait()
        self.init_robot()
    
    def hold(self):
        self.open()
        movej(posj([25.70, 23.15, 102.02, -72.06, 109.25, -49.19]), vel=VELOCITY, acc=ACC)
        mwait()     

        self.force_detect(force=0, min_force=6)

        self.close()
        time.sleep(0.1)

    def unhold(self):        
        self.open()
        self.init_robot()

    
    # def bring(self, get_keyword_result):
    #     tools = get_keyword_result        

    #     base_drop_pos = [175.31, 6.68, 293.24, 6.92, -179.73, 6.80]  # 기준 위치
    #     offset_x = 80  # x 방향 오프셋 (mm 단위)
    #     for i, tool in enumerate(tools):
    #         movej(posj([63.21, 15.56, 61.48, 0.05, 103.35, 63.43]), vel=VELOCITY, acc=ACC)
    #         mwait()            
    #         for attempt in range(2):
    #             target_pos = self.get_target_pos(tool.strip())
    #             if target_pos is None:
    #                 self.get_logger().warn("No target position")
    #                 movel([0, 75, 0, 0, 0, 0], vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_REL)
    #                 mwait()
    #             else:
    #                 self.get_logger().info("Tool detected")
    #                 self.move_stop_func(1)
    #                 break            

    #         if target_pos is None:
    #             self.get_logger().error(f"Cannot find tool: {tool}!")
    #             continue  # 다음 tool로 넘어감

    #         self.get_logger().info(f'{tool}')
    #         self.get_logger().info(f"Target position: {target_pos}")              

    #         self.pick(target_pos, tool, 196)          

    #         movej(posj([63.47, 33.25, 47.21, 0.16, 99.6, 62.45]), vel=VELOCITY, acc=ACC)
    #         mwait()            

    #         movej(posj([0, 0, 90, 0, 90, 0]), vel=VELOCITY, acc=ACC)
    #         mwait()            

    #         drop_pos = base_drop_pos.copy()
    #         drop_pos[0] += offset_x * i            

    #         self.get_logger().info(f"Moving to drop position: {drop_pos}")
    #         movel(posx(drop_pos), vel=VELOCITY, acc=ACC)
    #         mwait()            

    #         gripper.open_gripper()
    #         while gripper.get_status()[0]:
    #             time.sleep(0.5)
    #         mwait()            

    #         self.init_robot()

    # def clear(self):
    #     tools = [*pos_dic.keys()]
    #     task = []
        
    #     movel(posx([0, 0, 100, 0, 0, 0]), vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_REL)
    #     mwait()
        
    #     for _ in range(2):
    #         self.change_yolo_model(False)

    #         current_pos = get_current_posx()[0]
    #         while tools:
    #             self.get_logger().error(f'{tools}')
    #             tool = tools.pop()
    #             for pos in pos_dic[tool]:
    #                 target_pos = self.get_target_pos(tool)
    #                 if target_pos is None:
    #                     self.get_logger().warn(f'{tool} no detect')
    #                     task.append(tool)
    #                     continue

    #                 self.get_logger().info(f'{tool}')
    #                 self.get_logger().info(f"target position: {target_pos}")

    #                 self.pick(target_pos, tool, 290)
                    
    #                 self.get_logger().error('movel')
    #                 movel(posx(current_pos), vel=VELOCITY, acc=ACC)
    #                 mwait()
    #                 self.get_logger().error('movej')
    #                 movej(posj([63.47, 33.25, 47.21, 0.16, 99.6, 62.45]), vel=VELOCITY, acc=ACC)
    #                 mwait()
    #                 pos = pos + get_current_posx()[0][3:]
                    
    #                 movel(posx(pos), vel=VELOCITY, acc=ACC)
    #                 mwait()
                    
    #                 self.place(10)

    #                 movej(posj([63.47, 33.25, 47.21, 0.16, 99.6, 62.45]), vel=VELOCITY, acc=ACC)
    #                 mwait()
    #                 self.get_logger().error(str(get_current_posx()[0]))
    #                 movel(posx(current_pos), vel=VELOCITY, acc=ACC)
    #                 mwait()
            
    #         tools = [*task]

    #         movel(posx([0, -80, 0, 0, 0, 0]), vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_REL)
    #         mwait()
    #     self.init_robot()

    # def get_target_pos(self, target):
    #     self.get_position_request.target = target
    #     self.get_logger().info("call depth position service with object_detection node")
    #     get_position_future = self.get_position_client.call_async(
    #         self.get_position_request
    #     )
    #     rclpy.spin_until_future_complete(self, get_position_future)

    #     if get_position_future.result():
    #         result = get_position_future.result().depth_position.tolist()
    #         self.get_logger().info(f"Received depth position: {result}")
    #         if sum(result) == 0:
    #             print("No target position")
    #             return None

    #         gripper2cam_path = os.path.join(
    #             package_path, "resource", "T_gripper2camera.npy"
    #         )

    #         robot_posx = get_current_posx()[0]
    #         td_coord = self.transform_to_base(result, gripper2cam_path, robot_posx)

    #         if td_coord[2] and sum(td_coord) != 0:
    #             td_coord[2] += DEPTH_OFFSET  # DEPTH_OFFSET
    #             td_coord[2] = max(td_coord[2], MIN_DEPTH)  # MIN_DEPTH: float = 2.0

    #         target_pos = list(td_coord[:3]) + robot_posx[3:]

    #     return target_pos

    # def init_robot(self):
    #     JReady = [0, 0, 90, 0, 90, 0]
    #     movej(JReady, vel=VELOCITY, acc=ACC)
    #     gripper.open_gripper()
    #     mwait()
    
    # def pick(self, target_pos, tool, z):
    #     gripper.open_gripper()
    #     while gripper.get_status()[0]:
    #         time.sleep(0.5)
    #     mwait()

    #     target_pos[2] += 150
    #     set_tcp('realsence_tcp')
    #     movel(posx(target_pos), vel=VELOCITY, acc=ACC)
    #     mwait()

    #     self.change_yolo_model(True)
        
    #     amovesx([posx([0, 0, 0, 0, 0, 179]), posx([0, 0, 0, 0, 0, 179])], 
    #             vel=10, acc=0, time=25, mod=DR_MV_MOD_REL)
    #     target_pos = self.direciotn(tool)
        
    #     if tool in ['nutcup', 'voltcup']:
    #         z += 40    
    #     target_pos[2] = z
    #     movel(target_pos, vel=VELOCITY, acc=ACC)
    #     mwait()

    #     gripper.close_gripper()
    #     while gripper.get_status()[0]:
    #         time.sleep(0.5)
    #     mwait()    

    #     movel(posx([0, 0, 100, 0, 0, 0]), vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_REL)
    #     mwait()
    #     self.change_yolo_model(False)

    # # 기존 코드
    # def place(self, force_min):
    #     task_compliance_ctrl(stx=[100, 100, 100, 200, 200, 200])
    #     time.sleep(0.1)
    #     set_desired_force(fd=[0, 0, -10, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
    #     while check_force_condition(DR_AXIS_Z, min=force_min, ref=DR_BASE) == -1:
    #         continue
    #     release_force()
    #     release_compliance_ctrl()
                
    #     movel(posx([0, 0, 5, 0, 0, 0]), vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_REL)
    #     mwait()

    #     gripper.open_gripper()
    #     while gripper.get_status()[0]:
    #         time.sleep(0.5)
    #     mwait()

    # # 수정코드
    # # def place(self, place_to_pos):
    # #     place_to_pos[2] += 10 #높이 설정        #place_to_pos에서 받은 좌표로 이동
    # #     self.get_logger().info(f'place_to_pos: {place_to_pos}')
    # #     movel(posx(place_to_pos), vel=VELOCITY, acc=ACC)
    # #     mwait()    

    # #     self.get_logger().info('task_compliance_ctrl')
    # #     task_compliance_ctrl(stx=[100, 100, 100, 200, 200, 200])
    # #     time.sleep(0.1)
    # #     set_desired_force(fd=[0, 0, -10, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
    # #     while check_force_condition(DR_AXIS_Z, min=6, ref=DR_BASE) == -1:
    # #         continue
    # #     release_force()
    # #     release_compliance_ctrl()
        
    # #     movel([0, 0, 10, 0, 0, 0], vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_REL)
    # #     mwait()  

    # #     gripper.open_gripper()
    # #     while gripper.get_status()[0]:
    # #         time.sleep(0.5)
    # #     mwait()

    # def hold(self):
    #     movej(posj([25.70, 23.15, 102.02, -72.06, 109.25, -49.19]), vel=VELOCITY, acc=ACC)
    #     mwait()     

    #     task_compliance_ctrl(stx=[100, 100, 100, 200, 200, 200])
    #     time.sleep(0.1)
    #     set_desired_force(fd=[0, 0, 0, 0, 0, 0], dir=[1, 1, 1, 0, 0, 0], mod=DR_FC_MOD_REL)

    #     while True:
    #         if check_force_condition(DR_AXIS_X, min=6, ref=DR_BASE) == 0:
    #             release_force()
    #             release_compliance_ctrl()
    #             gripper.close_gripper()
    #             break        
    #     time.sleep(0.1)

    # def unhold(self):        
    #     # task_compliance_ctrl(stx=[100, 100, 100, 200, 200, 200])
    #     # time.sleep(0.1)
    #     # set_desired_force(fd=[0, 0, 0, 0, 0, 0], dir=[1, 1, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
    #     # while True:
    #     #     if check_force_condition(DR_AXIS_X, min=15, ref=DR_BASE) == 0:
    #     #         release_force()
    #     #         release_compliance_ctrl()
    #     #         gripper.open_gripper()
    #     #         break
    #     gripper.open_gripper()
    #     while gripper.get_status()[0]:
    #         time.sleep(0.5)
    #     mwait()
    #     self.init_robot()

    # def direciotn(self, target):
    #     self._detection_request.target = target
    #     self.get_logger().info("call depth position service with object_detection node")
    #     get_direction = self._detection_client.call_async(
    #         self._detection_request
    #     )
    #     rclpy.spin_until_future_complete(self, get_direction)

    #     if get_direction.result():
    #         result = get_direction.result().depth_position.tolist()
    #         self.get_logger().info(f"Received depth position: {result}")
    #         if sum(result) == 0:
    #             print("No target position")
    #             return None

    #     self.move_stop_func(1)
    #     set_tcp('')
    #     self.get_logger().info(f'2. tcp: {get_tcp()}')

    #     gripper2cam_path = os.path.join(
    #         package_path, "resource", "T_gripper2camera.npy"
    #     )

    #     time.sleep(2)
    #     robot_posx = get_current_posx()[0]
    #     td_coord = self.transform_to_base(result, gripper2cam_path, robot_posx)

    #     if td_coord[2] and sum(td_coord) != 0:
    #         td_coord[2] += DEPTH_OFFSET  # DEPTH_OFFSET
    #         td_coord[2] = max(td_coord[2], MIN_DEPTH)  # MIN_DEPTH: float = 2.0

    #     target_pos = list(td_coord[:3]) + robot_posx[3:]

    #     return target_pos


def main(args=None):
    node = RobotController()
    while rclpy.ok():
        # node.robot_control()
        node.bring(['screwdriver', '6wrench'])
        time.sleep(2)
        node.hold()
        time.sleep(5)
        node.unhold()
        time.sleep(2)
        node.clear()
    rclpy.shutdown()
    node.destroy_node()


if __name__ == "__main__":
    main()
