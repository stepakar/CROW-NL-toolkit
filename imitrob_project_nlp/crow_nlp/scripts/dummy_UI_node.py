import os
import sys
import traceback

import actionlib
import rospy
from crow_nlp.msg import SpeechUserInteractionAction, SpeechUserInteractionGoal, nlp_enum
from pkg_resources import resource_filename
import json
from opcua import ua
import argparse

def trace_exception(limit=2):
    exc_type, exc_value, exc_traceback = sys.exc_info()
    traceback.print_exception(exc_type, exc_value, exc_traceback, limit=limit, file=sys.stdout)
from nlp_crow.speech_recognition.SpeechProcessor import SpeechProcessor
from nlp_crow.utils.ServerConnect import ServerConnect
#from nlp_crow.utils.loading import Loading
from concurrent.futures import TimeoutError as ConcurrentTimeoutError

class UI(object):

    def __init__(self):


        self.action_client = actionlib.SimpleActionClient('SpeechUI', SpeechUserInteractionAction)

        try:
            self.action_client.wait_for_server(rospy.Duration(20))  # wait till the action server is up

            # rospy.on_shutdown(self.destroy)
        except rospy.ROSException as e:
            print(e)
            exit(22)

    def print_feedback(self, fb):
        print(fb)

    def get_program(self):
        req = SpeechUserInteractionGoal()
        req.request_type = nlp_enum.DETECT_PROGRAM
        res = self.action_client.send_goal(req, feedback_cb=self.print_feedback)
        self.action_client.wait_for_result(timeout=rospy.Duration.from_sec(10.0))
        if res == 3:
            data = self.action_client.get_result()
            print(data)

        print(res)

    def get_next_state(self):
        req = SpeechUserInteractionGoal()
        req.request_type = nlp_enum.SYSTEM_REQUEST
        req.user_msg = 'Please select your favorite horse color! The options are red, blue, and green.'
        res = self.action_client.send_goal(req, feedback_cb=self.print_feedback)
        self.action_client.wait_for_result(timeout=rospy.Duration.from_sec(10.0))
        if res == 3:
            data = self.action_client.get_result()
            print(data)


        print(res)

    def connect_to_server(self):
        root_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir)
        config_file = os.path.join(
            root_dir, "tests", "config.json")

        with open(config_file, "r") as f:
            config = json.load(f)

        storeAddress = False
        if not args.host or args.host == "last":
            self.address = config["last_address"]
        elif args.host == "local":
            self.address = config["local_address"]
        else:
            self.address = args.host
            storeAddress = True

        sc = ServerConnect(self.address)
        print(f"Connecting to {self.address}.")
        try:
            sc.connect()  # try to connect and setup necessary variables
        except ua.uaerrors.BadNoMatch:
            print("The requested variable does not exist on the OPC server!")
            trace_exception()
            sc.disconnect()
            sys.exit(-2)
        except KeyboardInterrupt:
            print("User requested interruption.")
            sc.disconnect()
            sys.exit(0)
        except Exception:
            print("Connecting to the server failed!")
            trace_exception()
            try:
                ServerConnect.disconnect()
            except Exception:
                pass
            sys.exit(-2)
        else:
            print('Connected to OPC server.')
            if storeAddress:
                config["last_address"] = self.address
                with open(config_file, "w") as f:
                    json.dump(config, f)

        if args.gain is not None:
            config["gain"] = args.gain
            with open(config_file, "w") as f:
                json.dump(config, f)
            self.gain = args.gain
        else:
            self.gain = config["gain"]
        return sc


    def prepare_setup(self):
        self.load_files()


    def load_files(self):
        # root_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)))
        # variants_file = os.path.join(root_dir, "utils", "state_description_next.json")
        root_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir)
        variants_file = os.path.join(
            root_dir, "utils", "state_description.json")
        #variants_file = resource_filename("crow_nlp", os.path.join("utils", "state_description.json"))
        with open(variants_file, "r", encoding="utf-8") as f:
            self.variants_list = json.load(f)

        # if self.USE_ACTIVE_STATE:
        #     next_states_file = os.path.join(
        #         root_dir, "utils", "next_states.json")
        #     #next_states_file = resource_filename("crow_nlp", os.path.join("utils", "next_states.json"))
        #     with open(next_states_file, "r", encoding="utf-8") as f:
        #         self.next_states_list = json.load(f)

        state_hints_file = os.path.join(
            root_dir, "utils", "state_hints.json")
        #state_hints_file = resource_filename("crow_nlp", os.path.join("utils", "state_hints.json"))
        with open(state_hints_file, "r", encoding="utf-8") as f:
            self.state_hints_list = json.load(f)

        responses_file = os.path.join(
            root_dir, "utils", "responses.json")
        #state_hints_file = resource_filename("crow_nlp", os.path.join("utils", "responses.json"))
        with open(responses_file, "r", encoding="utf-8") as f:
            self.responses = json.load(f)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("host", nargs="?", default="local",
                        help="OPC host server address. Use 'local' for a local host or 'last' for last used (non-local) address.")
    parser.add_argument("--configure_device", "-d", nargs="?", default=-2,
                        const=-1)  # default == False value, const == True value, some Natural int == device index
    parser.add_argument("--gain", "-g", default=None, type=float)
    parser.add_argument("--allow_interrupt", "-i", action="store_true", default=False)
    args = parser.parse_args()

    rospy.init_node('fake_ui')

    ui = UI()

    sc = ui.connect_to_server()

    ui.prepare_setup()

    SpeechProcessor.ALLOW_INTERRUPTIONS = args.allow_interrupt
    sp = SpeechProcessor(ui.address, gain=ui.gain)

    cdev = int(args.configure_device)
    if cdev < 0:
        cdev = True if cdev == -1 else False
    try:
        sp.init(configure_deivce=cdev)
    except AttributeError as ae:
        print(
            f"Sound initialization and calibration failed due to:\n'{ae}'\nThis is likely caused by missing PyAudio or incompatible version of it.\nExiting.")
        try:
            sc.disconnect()
        except Exception:
            pass
        sys.exit(-2)
    except KeyboardInterrupt:
        print("User requested interruption.")
        sc.disconnect()
        sys.exit(0)
    except Exception:
        trace_exception()
        try:
            sc.disconnect()
        except Exception:
            pass
        sys.exit(-2)

    SEL = 1
    try:
        # if SEL == 1:
        #     ui.get_next_state()
        # elif SEL == 2:
        #     ui.get_program()
        sp.run()  # run the main detection loop
    except OSError:
        print("There was an OS Error, likely a problem while communicating with the server. The error message was:")
        trace_exception()
    except (ConnectionAbortedError, ConcurrentTimeoutError):
        print("The connection was aborted by the server! (possibly)")
        trace_exception()
    except KeyboardInterrupt:
        print("User requested interruption.")
    except Exception:
        print("An unknown error raised:")
        trace_exception(20)
    finally:
        try:
            sc.disconnect()  # always disconnect at the end
        except Exception:
            trace_exception()

    # ui.get_program()
    #
    # ui.get_next_state()