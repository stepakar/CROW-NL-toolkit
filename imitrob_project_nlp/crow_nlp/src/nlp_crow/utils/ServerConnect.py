
from opcua import Client  # import the Client class
from opcua import ua
import json
import os
import sys
try:
    from inputimeout import inputimeout, TimeoutOccurred
    INPUTIMEOUT_IMPORT_SUCCESS = True
except ImportError:
    INPUTIMEOUT_IMPORT_SUCCESS = False
from pkg_resources import resource_filename
import traceback
from select import select

# def trace_exception(limit=2):
#     exc_type, exc_value, exc_traceback = sys.exc_info()
#     traceback.print_exception(exc_type, exc_value, exc_traceback, limit=limit, file=sys.stdout)
#
# def kbhit():
#     dr,dw,de = select([sys.stdin], [], [], 0)
#     return dr != []

class ServerConnect():
    # WRONG_INPUT_STATE = 0
    # SEND_ERRORS = False  # whether to send error to OPC server or just show them in console
    # USE_ACTIVE_STATE = False
    # STRIP_ACCENTS = True  # whether to remove accents before comparing recognized text and possible choices
    # ALLOW_INTERRUPTIONS = False  # whether the user can interupt TTS playback
    #
    # MICROPHONE_WAIT_TIMEOUT = 5  # time to wait for any non-zero audio from mic
    # LISTENING_START_TIMEOUT = 5  # time to start speaking after recognition is ran
    # PHRASE_TIMEOUT = 4  # maximum length of phrase before listening is cut off
    # CALIBRATION_TIME = 1  # time to spend calibrating the microphone bgr energy levels

    def __init__(self, serverAdress, gain: int = 1.0):
        # instantiate client object; make sure address and port are correct
        self.client = Client(serverAdress)
        # self.shouldProcessInput = True
        # self.LANG = 'cs'
        # self.recognizer = sr.Recognizer()
        # self.fast_recognizer = sr.Recognizer()
        # self.fast_recognizer.non_speaking_duration = 0.05
        # self.fast_recognizer.pause_threshold = 0.15
        # self.recText = RawNLParser()
        # self.lastRecognitionFailed = False
        # self.repeatChoices = False


    def connect(self):
        self.client.connect()  # connect to server
        root = self.client.nodes.root  # get the root entity
        print("Connected to server and successfully retrieved the root.")

        # Retreive some objects and variables
        dataObj = root.get_child(["0:Objects", "4:DATA"])

        # a horse
        horse = dataObj.get_child("4:horse")

        # mic_active is the "You can now try recognizing speech" variable
        self.mic_active = horse.get_child("4:request")

        # the string where responses (errors and such) should be sent
        self.response_string = horse.get_child("4:response")

        # the state change request variable
        self.request = horse.get_child(["4:next_state_choice", "4:request"])
        # self.request.set_value(True)
        # the number of the state to be changed to
        self.request_state_num = horse.get_child(["4:next_state_choice", "4:state_num"])

        # the variable with next state choices
        self.next_state_possibilities = horse.get_child("4:next_state_possibilities")
        # self.client.load_type_definitions()

        # the variable with the current state
        self.actual_state_number = horse.get_child(["4:actual_state", "4:number"])

        self.sub = self.client.create_subscription(100, self)  # create subscription
        self.handle = self.sub.subscribe_data_change(self.mic_active)

    def send_state(self, state_num):
        """
        Changes state to the specified number and sets "request" to True.
        """
        dt = ua.DataValue(ua.Variant(state_num, ua.VariantType.Int16))
        self.request_state_num.set_value(dt)
        self.request.set_value(True)

    # def __extract_directive(self, string):
    #     result = self.hint_directive_re.split(string)
    #     if len(result) > 1:
    #         return result[0], result[1]
    #     else:
    #         return result[0], ""

    def disconnect(self):
        try:
            self.sub.unsubscribe(self.handle)  # cancel subscription
            self.sub.delete()
        except AttributeError:
            pass  # not subscribed, yet
        except Exception:
            print("Error trying to unsubscribe from the mic_active variable!")
        self.client.disconnect()