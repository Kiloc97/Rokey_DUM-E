import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
import threading

class WakeupKey(Node):
    def __init__(self):
        super().__init__('wakeup_node')
        self.wake_server = self.create_client(SetBool, 'wakeup_teleop')

        while not self.wake_server.wait_for_service(timeout_sec=3.0):
            self.get_logger().info("Waiting for wakeup service...")

        self.wakeup_request = SetBool.Request()
        threading.Thread(target=self.input_thread, daemon=True).start()
        

        # self.input_thread = threading.Thread(target=self.get_key_input)
        # self.input_thread.start()

    # def get_key_input(self):
    #     while self.key != 's':
    #         self.key = input("Press 's' to trigger wakeup: ")

    #     self.wakeup_key_callback()

    def input_thread(self):
        while True:
            key = input("Press 's' to activated: ")
            self.send_command(key)

    def send_command(self, key):
        if key == 's':
            self.wakeup_request.data = True
        else:
            self.wakeup_request.data = False
        future = self.wake_server.call_async(self.wakeup_request)
        future.add_done_callback(self.callback_future)
    
    def callback_future(self, future):
        response = future.result()
        if response.success:
            self.get_logger().info(response.message)
        else:
            self.get_logger().warn(response.message)
        

def main(args=None):
    rclpy.init(args=args)
    user_input = WakeupKey()
    try:
        rclpy.spin(user_input)
    except KeyboardInterrupt:
        pass
    finally:
        user_input.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()