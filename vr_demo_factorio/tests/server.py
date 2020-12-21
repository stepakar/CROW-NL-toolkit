import os.path
try:
    from IPython import embed
except ImportError:
    import code

    def embed():
        vars = globals()
        vars.update(locals())
        shell = code.InteractiveConsole(vars)
        shell.interact()

from opcua import Server, ua
import time
import msvcrt
import json
import sys
import traceback
import argparse


class SubHandler(object):
    """
    Subscription handler handles subscriptions (e.g. receives data change events).
    """
    def __init__(self, parent):
        self.parent = parent

    def datachange_notification(self, node, val, data):
        """
        Subscription handler can be any object as long as it
        has this method with these arguments.
        """
        # print(dir(node))
        nodeName = node.get_display_name().to_string()
        if "request" in nodeName:
            if val:
                print(
                    f"State change requested")
                self.parent.change_state(self.parent.state_num.get_value())
                data.monitored_item.Value.Value.Value = False
                print("Setting change request back to false.")
            else:
                print("State change requested turned off.")
        elif "response" in nodeName:
            print(f"A response was sent:\n{val}")
        elif "number" in nodeName:
            self.parent.check_current_state()
        else:
            print(f"Node {nodeName} sent {val}")

    def event_notification(self, event):
        print("New event received: ", event)


def readInput(caption, default="", timeout=5):
    start_time = time.time()
    # sys.stdout.write('%s(%s):' % (caption, default))
    inp = ''
    while True:
        if msvcrt.kbhit():
            char = msvcrt.getche()
            if ord(char) == 13:  # enter_key
                break
            elif ord(char) >= 32:  # space_char
                inp = chr(ord(char))
                break
        if len(inp) == 0 and (time.time() - start_time) > timeout:
            break

    print('')  # needed to move to next line
    if len(inp) > 0:
        return inp
    else:
        return default


class TestServer:
    lang = "cs"

    def __init__(self, endpoint, name):
        parser = argparse.ArgumentParser()
        parser.add_argument("--start_state", "-s", type=int, default=0)
        args = parser.parse_args()
        start_state = args.start_state

        root_dir = os.path.join(os.path.dirname(__file__), os.path.pardir).replace('\\', '/')
        print(root_dir)
        variants_file = os.path.join(root_dir, "speech_vr/utils", "next_states.json").replace('\\', '/')
        variants_file_desc = os.path.join(root_dir, "speech_vr/utils", "state_description.json").replace('\\', '/')
        with open(variants_file, encoding="utf-8") as f:
            self.state_descriptions = json.load(f, encoding="utf-8")[self.lang]
        with open(variants_file_desc, encoding="utf-8") as f:
            self.next_state_description = json.load(f, encoding="utf-8")[self.lang]

        self.server = Server()

        # self.server.import_xml(model_filepath)  # normally, definition would be loaded here

        self.server.set_endpoint(endpoint)  # specify the endpoint (address)
        self.server.set_server_name(name)  # specify server name (whatever)

        objects = self.server.get_objects_node()  # get the object containing entity

        myobj = objects.add_object(4, "DATA")  # create new object

        horse = myobj.add_object(4, "horse")  # create new object

        self.mic_active = horse.add_variable(4, "request", False)
        self.mic_active.set_writable()

        response_string = horse.add_variable(4, "response", "")
        response_string.set_writable()

        self.actual_state = horse.add_variable(4, "actual_state", True)
        self.actual_state_num = self.actual_state.add_variable(4, "number", 0)
        self.actual_state.add_variable(4, "ready", True)

        next_state_choice = horse.add_variable(4, "next_state_choice", True)
        self.request_var = next_state_choice.add_variable(4, "request", False)
        self.request_var.set_writable()
        dt = ua.DataValue(ua.Variant(0, ua.VariantType.Int16))  # to ensure correct data type
        self.state_num = next_state_choice.add_variable(4, "state_num", dt)
        self.state_num.set_writable()

        self.next_state_possibilities = horse.add_variable(4, "next_state_possibilities", [1, 70, 80])
        self.change_state(start_state)

        print(f"Server is in state {self.actual_state_num.get_value()} and the possibilities are {type(self.next_state_possibilities.get_value())}")

        self.start()  # start the server

        self.mic_active.set_value(False)
        subHandler = SubHandler(self)
        sub = self.server.create_subscription(500, subHandler)
        handle = sub.subscribe_data_change(self.request_var)
        handle_response = sub.subscribe_data_change(response_string)
        handle_actual = sub.subscribe_data_change(self.actual_state_num)

        # while True:
        try:
            print("Server started.")
            while True:
                # print("mic mic", self.mic_active.get_value())
                # time.sleep(10)  # the variable "mic_active" will be changed periodically every 10 seconds in this loop

                # change the mic_active (if True -> False, if False -> True)
                # mic_active.set_value(not mic_active.get_value())

                inp = readInput("", timeout=10)
                if inp == "q":
                    break
                elif inp == "r":
                    self.mic_active.set_value(True)
                    print(f"Variable 'mic_active' changed to {self.mic_active.get_value()}.")
                elif inp == "w":
                    self.mic_active.set_value(False)
                    print(f"Variable 'mic_active' changed to {self.mic_active.get_value()}.")
        except KeyboardInterrupt:
            print("User requested interruption.")
        except Exception:
            exc_type, exc_value, exc_traceback = sys.exc_info()
            traceback.print_exception(exc_type, exc_value, exc_traceback, limit=2, file=sys.stdout)

        sub.unsubscribe(handle)  # cancel subscription
        sub.unsubscribe(handle_response)  # cancel subscription
        sub.unsubscribe(handle_actual)  # cancel subscription
        sub.delete()

    def change_state(self, next_state):
        current_state = str(self.actual_state_num.get_value())
        print(f"Changing state from {current_state} to {next_state}.")
        if current_state in self.state_descriptions:
            self.next_state_possibilities.set_value(self.state_descriptions[current_state])
        self.actual_state_num.set_value(next_state)

    def check_current_state(self):
        state = str(self.actual_state_num.get_value())
        if state in self.state_descriptions:
            next_states = self.state_descriptions[state]
            # self.next_state_possibilities = horse.add_variable(4, "next_state_possibilities", next_states)
            self.next_state_possibilities.set_value(next_states)
            # self.next_state_possibilities.set_value = next_states
            if len(next_states) == 1:
                next_state = next_states[0]
                # print(self.next_state_description)
                if str(next_state) in self.next_state_description:
                    print(
                        f"Current state {state} has single next state: {next_state}")
                    print(self.next_state_possibilities)
                    print(self.actual_state)
                    self.mic_active.set_value(True)
                else:
                    print(f"Current state {state} does not require user input, skipping to next state: {next_state}")
                    self.actual_state_num.set_value(next_state)
                    self.mic_active.set_value(False)
            else:
                print(f"Current state {state} has multiple choices: {next_states}")
                self.mic_active.set_value(True)
        else:
            print("Current state is not in the state description file!")

    def start(self):
        self.server.start()
        return self.server

    def stop(self):
        self.server.stop()


if __name__ == '__main__':
    script_dir = os.path.dirname(__file__)
    server = TestServer(
        "opc.tcp://localhost:4840/",
        "FreeOpcUa Server")

    # needs a more meaningful way of stopping
    server.stop()
