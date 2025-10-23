import numpy as np
import rclpy
from rclpy.node import Node
from typing import Any, Callable, Optional, Tuple

from ament_index_python.packages import get_package_share_directory
from od_msg.srv import SrvDepthPosition
from object_detection.realsense import ImgNode
from object_detection.yolo import YoloModel
from std_srvs.srv import SetBool

from rclpy.action import ActionServer
# from interface.action import a


# PACKAGE_NAME = 'pick_and_place_text'
# PACKAGE_PATH = get_package_share_directory(PACKAGE_NAME)


class ObjectDetectionNode(Node):
    def __init__(self, model_name = 'yolo'):
        super().__init__('object_detection_node')
        self.img_node = ImgNode()
        self.model = self._load_model(model_name)
        self.intrinsics = self._wait_for_valid_data(
            self.img_node.get_camera_intrinsic, "camera intrinsics"
        )
        self.create_service(
            SrvDepthPosition,
            'get_3d_position',
            self.handle_get_depth
        )
        self.get_logger().info("ObjectDetectionNode initialized.")
        self.create_service(
            SetBool,
            'change_yolo_model',
            self.handle_model_change)
        
        self._action_server = ActionServer(
            self, a, 'direction', self.action_detectoin_callback
        )
        self.get_logger().info('Action Server Initialized')

    def _load_model(self, name):
        """모델 이름에 따라 인스턴스를 반환합니다."""
        if name.lower() == 'yolo':
            return YoloModel()
        raise ValueError(f"Unsupported model: {name}")

    def handle_get_depth(self, request, response):
        """클라이언트 요청을 처리해 3D 좌표를 반환합니다."""
        self.get_logger().info(f"Received request: {request}")
        coords = self._compute_position(request.target)
        response.depth_position = [float(x) for x in coords]
        return response

    def _compute_position(self, target):
        """이미지를 처리해 객체의 카메라 좌표를 계산합니다."""
        rclpy.spin_once(self.img_node)

        box, score = self.model.get_best_detection(self.img_node, target)
        if box is None or score is None or score < 0.8:
            self.get_logger().warn("No detection found.")
            return 0.0, 0.0, 0.0
        
        self.get_logger().info(f"Detection: box={box}, score={score}")
        cx, cy = map(int, [(box[0] + box[2]) / 2, (box[1] + box[3]) / 2])
        cz = self._get_depth(cx, cy)
        if cz is None:
            self.get_logger().warn("Depth out of range.")
            return 0.0, 0.0, 0.0

        return self._pixel_to_camera_coords(cx, cy, cz)

    def _get_depth(self, x, y):
        """픽셀 좌표의 depth 값을 안전하게 읽어옵니다."""
        frame = self._wait_for_valid_data(self.img_node.get_depth_frame, "depth frame")
        try:
            return frame[y, x]
        except IndexError:
            self.get_logger().warn(f"Coordinates ({x},{y}) out of range.")
            return None

    def _wait_for_valid_data(self, getter, description):
        """getter 함수가 유효한 데이터를 반환할 때까지 spin 하며 재시도합니다."""
        data = getter()
        while data is None or (isinstance(data, np.ndarray) and not data.any()):
            rclpy.spin_once(self.img_node)
            self.get_logger().info(f"Retry getting {description}.")
            data = getter()
        return data

    def _pixel_to_camera_coords(self, x, y, z):
        """픽셀 좌표와 intrinsics를 이용해 카메라 좌표계로 변환합니다."""
        fx = self.intrinsics['fx']
        fy = self.intrinsics['fy']
        ppx = self.intrinsics['ppx']
        ppy = self.intrinsics['ppy']
        return (
            (x - ppx) * z / fx,
            (y - ppy) * z / fy,
            z    
        )

    def handle_model_change(self, request, response):
        try:
            if request.data == False:
                self.model.switch_to_best_model()
            elif request.data == True:
                self.model.switch_to_over_model()
            else:
                raise ValueError(f"Unknown model name: {request.data}")
            
            response.success = True
            response.message = f"Model changed to {request.data}"
            self.get_logger().info(response.message)
        except Exception as e:
            response.success = False
            response.message = str(e)
            self.get_logger().error(response.message)
        return response
    
    def action_detectoin_callback(self, goal_hamdle):
        goal = goal_hamdle.request.target
        self.get_logger().info(f'Action Server: Received goal request({goal})')
        
        while True:
            coords = self._compute_position(goal)
            if sum(coords) == 0:
                continue
            break
        self.get_logger().info('Action Server: Direction Detect')

        # direction detection 
        result = a.Result()
        result.depth_position = [float(x) for x in coords]
        goal_hamdle.succeed()
        self.get_logger().info("Action Server: Goal succeeded")
        
        return result


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
