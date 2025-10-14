#!/usr/bin/env python3
from flask import Flask
from ask_sdk_core.skill_builder import SkillBuilder
from flask_ask_sdk.skill_adapter import SkillAdapter
from ask_sdk_core.utils import is_request_type, is_intent_name
from ask_sdk_core.handler_input import HandlerInput
from ask_sdk_model import Response
from ask_sdk_model.ui import SimpleCard
from ask_sdk_core.dispatch_components import AbstractRequestHandler, AbstractExceptionHandler
from arduinobot_msgs.action import ArduinobotTask
import rclpy
from rclpy.node import Node
import threading
from rclpy.action import ActionClient

threading.Thread(target=lambda: rclpy.init()).start()
action_client = ActionClient(Node('alexa_interface'), ArduinobotTask, "task_server")

app = Flask(__name__)


class LaunchRequestHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool
        return is_request_type("LaunchRequest")(handler_input)

    def handle(self, handler_input):
        # type: (HandlerInput) -> Response
        speech_text = "Hi, how can we help?"

        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard("Online", speech_text)).set_should_end_session(
            False)

        goal = ArduinobotTask.Goal()
        goal.task_number = 0
        action_client.send_goal_async(goal)

        return handler_input.response_builder.response


class PickIntentHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool
        return is_intent_name("PickIntent")(handler_input)

    def handle(self, handler_input):
        # type: (HandlerInput) -> Response
        speech_text = "Ok, I'm moving"

        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard("Pick", speech_text)).set_should_end_session(
            True)

        goal = ArduinobotTask.Goal()
        goal.task_number = 1
        action_client.send_goal_async(goal)

        return handler_input.response_builder.response


class SleepIntentHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool
        return is_intent_name("SleepIntent")(handler_input)

    def handle(self, handler_input):
        # type: (HandlerInput) -> Response
        speech_text = "Ok, see you later"

        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard("Sleep", speech_text)).set_should_end_session(
            True)

        goal = ArduinobotTask.Goal()
        goal.task_number = 2
        action_client.send_goal_async(goal)

        return handler_input.response_builder.response


class WakeIntentHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool
        return is_intent_name("WakeIntent")(handler_input)

    def handle(self, handler_input):
        # type: (HandlerInput) -> Response
        speech_text = "Hi, I am ready"

        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard("Wake", speech_text)).set_should_end_session(
            True)
            
        goal = ArduinobotTask.Goal()
        goal.task_number = 0
        action_client.send_goal_async(goal)

        return handler_input.response_builder.response


class DetectObjectsIntentHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool
        return is_intent_name("DetectObjectsIntent")(handler_input)

    def handle(self, handler_input):
        # type: (HandlerInput) -> Response
        speech_text = "I'm scanning for objects now"

        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard("Detect Objects", speech_text)).set_should_end_session(
            True)

        goal = ArduinobotTask.Goal()
        goal.task_number = 3  # Scanning mode
        action_client.send_goal_async(goal)

        return handler_input.response_builder.response


class PickObjectIntentHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool
        return is_intent_name("PickObjectIntent")(handler_input)

    def handle(self, handler_input):
        # type: (HandlerInput) -> Response
        speech_text = "I'm picking up the detected object"

        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard("Pick Object", speech_text)).set_should_end_session(
            True)

        goal = ArduinobotTask.Goal()
        goal.task_number = 4  # Pick detected object
        action_client.send_goal_async(goal)

        return handler_input.response_builder.response


class PlaceObjectIntentHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool
        return is_intent_name("PlaceObjectIntent")(handler_input)

    def handle(self, handler_input):
        # type: (HandlerInput) -> Response
        speech_text = "I'm placing the object in the drop zone"

        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard("Place Object", speech_text)).set_should_end_session(
            True)

        goal = ArduinobotTask.Goal()
        goal.task_number = 5  # Place object
        action_client.send_goal_async(goal)

        return handler_input.response_builder.response


class TestPositionIntentHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool
        return is_intent_name("TestPositionIntent")(handler_input)

    def handle(self, handler_input):
        # type: (HandlerInput) -> Response
        speech_text = "Moving to test scanning position"

        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard("Test Position", speech_text)).set_should_end_session(
            True)

        goal = ArduinobotTask.Goal()
        goal.task_number = 6  # Test position
        action_client.send_goal_async(goal)

        return handler_input.response_builder.response


class ScanRedObjectsIntentHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool
        return is_intent_name("ScanRedObjectsIntent")(handler_input)

    def handle(self, handler_input):
        # type: (HandlerInput) -> Response
        speech_text = "I'm scanning for red objects now"

        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard("Scan Red Objects", speech_text)).set_should_end_session(
            True)

        goal = ArduinobotTask.Goal()
        goal.task_number = 7  # Scan for red objects
        action_client.send_goal_async(goal)

        return handler_input.response_builder.response


class ScanBlueObjectsIntentHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool
        return is_intent_name("ScanBlueObjectsIntent")(handler_input)

    def handle(self, handler_input):
        # type: (HandlerInput) -> Response
        speech_text = "I'm scanning for blue objects now"

        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard("Scan Blue Objects", speech_text)).set_should_end_session(
            True)

        goal = ArduinobotTask.Goal()
        goal.task_number = 8  # Scan for blue objects
        action_client.send_goal_async(goal)

        return handler_input.response_builder.response


class ScanGreenObjectsIntentHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool
        return is_intent_name("ScanGreenObjectsIntent")(handler_input)

    def handle(self, handler_input):
        # type: (HandlerInput) -> Response
        speech_text = "I'm scanning for green objects now"

        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard("Scan Green Objects", speech_text)).set_should_end_session(
            True)

        goal = ArduinobotTask.Goal()
        goal.task_number = 9  # Scan for green objects
        action_client.send_goal_async(goal)

        return handler_input.response_builder.response


class PickAndPlaceIntentHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool
        return is_intent_name("PickAndPlaceIntent")(handler_input)

    def handle(self, handler_input):
        # type: (HandlerInput) -> Response
        # Extract color slot from the intent
        slots = handler_input.request_envelope.request.intent.slots
        color = "blue"  # Default to blue
        
        if "color" in slots and slots["color"].value:
            color = slots["color"].value.lower()
        
        speech_text = f"I'm performing a pick and place operation for {color} objects"

        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard("Pick and Place", speech_text)).set_should_end_session(
            True)

        goal = ArduinobotTask.Goal()
        # Map colors to task numbers
        color_task_map = {
            "red": 7,
            "blue": 8,
            "green": 9
        }
        goal.task_number = color_task_map.get(color, 8)  # Default to blue (task 8)
        action_client.send_goal_async(goal)

        return handler_input.response_builder.response


class HomePositionIntentHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool
        return is_intent_name("HomePositionIntent")(handler_input)

    def handle(self, handler_input):
        # type: (HandlerInput) -> Response
        speech_text = "Moving to home position"

        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard("Home Position", speech_text)).set_should_end_session(
            True)

        goal = ArduinobotTask.Goal()
        goal.task_number = 0  # Home position
        action_client.send_goal_async(goal)

        return handler_input.response_builder.response


class AllExceptionHandler(AbstractExceptionHandler):

    def can_handle(self, handler_input, exception):
        # type: (HandlerInput, Exception) -> bool
        return True

    def handle(self, handler_input, exception):
        # type: (HandlerInput, Exception) -> Response

        speech = "Hmm, I don't know that. Can you please say it again?"
        handler_input.response_builder.speak(speech).ask(speech)
        return handler_input.response_builder.response


skill_builder = SkillBuilder()
skill_builder.add_request_handler(LaunchRequestHandler())
skill_builder.add_request_handler(PickIntentHandler())
skill_builder.add_request_handler(SleepIntentHandler())
skill_builder.add_request_handler(WakeIntentHandler())
skill_builder.add_request_handler(DetectObjectsIntentHandler())
skill_builder.add_request_handler(PickObjectIntentHandler())
skill_builder.add_request_handler(PlaceObjectIntentHandler())
skill_builder.add_request_handler(TestPositionIntentHandler())
skill_builder.add_request_handler(ScanRedObjectsIntentHandler())
skill_builder.add_request_handler(ScanBlueObjectsIntentHandler())
skill_builder.add_request_handler(ScanGreenObjectsIntentHandler())
skill_builder.add_request_handler(PickAndPlaceIntentHandler())
skill_builder.add_request_handler(HomePositionIntentHandler())
skill_builder.add_exception_handler(AllExceptionHandler())


skill_adapter = SkillAdapter(
    skill=skill_builder.create(), 
    skill_id="amzn1.ask.skill.2bd29ade-0677-4dd0-b98a-200741363d9e",
    app=app)


skill_adapter.register(app=app, route="/")


if __name__ == '__main__':
    app.run()