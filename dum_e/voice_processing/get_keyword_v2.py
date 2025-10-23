# ros2 service call /get_keyword std_srvs/srv/Trigger "{}"

import os
import rclpy
import pyaudio
import threading
from time import sleep
from rclpy.node import Node

from ament_index_python.packages import get_package_share_directory
from dotenv import load_dotenv
from langchain.chat_models import ChatOpenAI
from langchain.prompts import PromptTemplate
from langchain.chains import LLMChain

from std_srvs.srv import Trigger
from voice_processing.MicController import MicController, MicConfig

from voice_processing.wakeup_word import WakeupWord
from voice_processing.stt import STT

from dum_e_interface.srv import Command

from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import SetBool
import threading
import time

import subprocess

############ Package Path & Environment Setting ############
current_dir = os.getcwd()
package_path = get_package_share_directory("dum_e")

is_laod = load_dotenv(dotenv_path=os.path.join(f"{package_path}/resource/.env"))
openai_api_key = os.getenv("OPENAI_API_KEY")

############ AI Processor ############
# class AIProcessor:
#     def __init__(self):



############ GetKeyword Node ############
class GetKeyword(Node):
    def __init__(self):
        appimage_path = os.path.join(package_path, 'resource/appimage_file.AppImage')
        # 권한 부여
        os.chmod(appimage_path, 0o755)
        # 비동기 실행 (백그라운드)
        self.proc = subprocess.Popen([appimage_path])

        self.llm = ChatOpenAI(
            model="gpt-4o", temperature=0.5, openai_api_key=openai_api_key
        )

        prompt_content = """
            You have to extract spcific tools and destination from User's sentence.
            I'm going to tip $200 for a better solution!
            Take a deep breath and work on this step by step.
            ###Instruction###
            You will be penalized if you miss one of the instructions below
             - You must accurately extract the tasks included in the following task list from the sentence.
             - You must accurately extract the tools included in the following tool list from the sentence.
             - You must extract destination of the tool(where you're told to move) in the sentence.
            ###Tasklist###
             - [bring, clear, hold]
            ###Toollist###
             - ['hammer, monkey, 6wrench, screwdriver, pipewrench, wrench, knief, voltcup, nutcup']
            ###OutputFormat###
             - You must follow following format : [task / tool1 tool2 ...]
             - Tools and postition must be classified by blank.
             - If there is no tool, the front must be empty without space, and if there is no destination, the back of '/' must be empty without space.
             - The sequence of tools and destinations must follows the sequence of appearance.
            ###SpecialRules###
             - If there is no clear tool name but it can be inferred in the context (Example: "nailing" → hammer), deduce as much as possible from the items in the list and return them.
             - If multiple tools and destinations appear at the same time, match them exactly for each and output them in order.
            ###Example###
             - Input : "Bring the hammer."
            Output : bring / hammer
             - Input : "Bring the hammer and wrench on the left."
            Output : bring / hammer wrench
             - Input : "Bring me the hammer on the left."
            Output : bring / hammer
             - Input : "Bring me something that can be nailed down"
            Output : bring / hammer
             - Input : "Bring me something to tighten the bolt"
            Output : bring / screwdriver monkey /
             - Input : "Bring me something to tighten the nut"
            Output : bring / 6wrench /
             - Input : "Bring me something suitable to cut something "
            Output : bring / knife /
             - Input : "Bring me something to tighten the bolt from above"
            Output : bring / screwdriver /
             - Input : "Put hammer on pos2 and put screwdriver on pos1"
            Output : bring / hammer screwdriver
             - Input : "Please organize it"
            Output : clear / /
             - Input : "hammer를 pos1에 가져다 놔"
            Output : bring / hammer
             - Input : "왼쪽에 있는 해머와 wrench를 pos1에 넣어줘"
            Output : bring / hammer wrench
             - Input : "왼쪽에 있는 hammer를줘"
            Output : bring / hammer
             - Input : "왼쪽에 있는 못 박을 수 있는것을 줘"
            Output : bring / hammer screwdriver
             - Input : '정리해줘'
            Output : clear /
            ###Input###
            "{user_input}"
            This is very important to my career.
        """

        tigger_prompt_content = """
            You have to extract spcific tools and destination from User's sentence.
            I'm going to tip $200 for a better solution!
            Take a deep breath and work on this step by step.
            ###Instruction###
            You will be penalized if you miss one of the instructions below
             - You must accurately extract the tasks included in the following task list from the sentence.
             - You must accurately extract the tools included in the following tool list from the sentence.
             - You must extract destination of the tool(where you're told to move) in the sentence.
            ###Tasklist###
             - [bring, clear, hold]
            ###OutputFormat###
             - You must follow following format : [task / tool1 tool2 ...]
             - Tools and postition must be classified by blank.
             - If there is no tool, the front must be empty without space, and if there is no destination, the back of '/' must be empty without space.
             - The sequence of tools and destinations must follows the sequence of appearance.
            ###SpecialRules###
             - If there is no clear tool name but it can be inferred in the context (Example: "nailing" → hammer), deduce as much as possible from the items in the list and return them.
             - If multiple tools and destinations appear at the same time, match them exactly for each and output them in order.
            ###Example###
             - Input : "Bring the hammer."
            Output : bring / hammer
            ###Input###
            "{user_input}"
            This is very important to my career.
        """

        self.prompt_template = PromptTemplate(
            input_variables=["user_input"], template=prompt_content
        )
        self.key_lang_chain = LLMChain(llm=self.llm, prompt=self.prompt_template)

        self.tigger_prompt_content = PromptTemplate(
            input_variables=["user_input"], template=tigger_prompt_content
        )
        self.trigger_lang_chain = LLMChain(llm=self.llm, prompt=self.tigger_prompt_content)
        self.stt = STT(openai_api_key=openai_api_key)


        super().__init__("get_keyword_node")
        # 오디오 설정
        mic_config = MicConfig(
            chunk=12000,
            rate=48000,
            channels=1,
            record_seconds=5,
            fmt=pyaudio.paInt16,
            device_index=10,
            buffer_size=24000,
        )
        self.mic_controller = MicController(config=mic_config)
        # self.ai_processor = AIProcessor()

        self.get_logger().info("MicRecorderNode initialized.")
        self.get_logger().info("wait for client's request...")
        self.get_keyword_srv = self.create_service(
            Command, "get_keyword", self.get_keyword
        )

        self.wake_server = self.create_service(SetBool, 'wakeup_teleop', self.wakeup_callback)
        
        self.wakeup_word = WakeupWord(mic_config.buffer_size)

        self.teleop_event = threading.Event()
        
        self.wakeup = False

        ##########################################
        self.wakeup_detected = False
        self.s_detected = False
        self.trigger = False

    def extract_keyword(self, lang_chain, output_message):
        response = lang_chain.invoke({"user_input": output_message})
        result = response["text"]
        self.get_logger().info(f"LLM Output: {result}")

        return result

    def get_keyword(self, request, response):  # 요청과 응답 객체를 받아야 함
        try:
            print("open stream")
            self.mic_controller.open_stream()
            self.wakeup_word.set_stream(self.mic_controller.stream)
        except OSError:
            self.get_logger().error("Error: Failed to open audio stream")
            self.get_logger().error("please check your device index")
            return None

        while not self.wakeup_word.is_wakeup():
            time.sleep(1)
            pass
        
        while True:
            # STT --> Keword Extract --> Embedding
            output_message = self.stt.speech2text()
            self.get_logger().info(f'input: {output_message}')
            trigger_message = self.stt.speech2text()
            if trigger_message:
                self.trigger = SetBool(self.extract_keyword(self.trigger_lang_chain, trigger_message))
                if self.trigger:
                    self.trigger = False
                    break

        result = self.extract_keyword(self.key_lang_chain, output_message)

        try:
            task, object = result.strip().split("/")
        except ValueError:
            self.get_logger().error(f"Invalid format: {result}")
        
        task = task.strip().split()[0]
        object = object.replace(",", " ").split() if object.strip() else []

        # 응답 객체 설정
        response.success = True
        response.task = task
        response.object = object
        return response

    def wakeup_callback(self, request, response):
        if request.data:
            response.success = True
            response.message = 'Recoding Start...'
            self.wakeup = True
        else:
            response.success = False
            response.message = 'Incorrect key!'
            self.wakeup = False

        return response


def main():
    rclpy.init()
    node = GetKeyword()
    excutor = MultiThreadedExecutor()
    excutor.add_node(node)
    try:
        excutor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
