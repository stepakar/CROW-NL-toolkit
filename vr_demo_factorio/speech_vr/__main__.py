import logging

import getch
import gtts
import rospy
from crow_nlp.msg import SentenceProgram
from database_com.msg import DBOperation
from database_com.srv import AddObjectRequest, AddObjectResponse, GetDatabaseRequest, SendDatabaseRequest, AddObject, \
    SendDatabase, GetDatabase
from playsound import playsound

from nlp_crow.database.Database import State
from nlp_crow.database.DatabaseAPI import DatabaseAPI, db
from nlp_crow.processing.NLProcessor import NLProcessor
from nlp_crow.processing.ProgramRunner import ProgramRunner
from nlp_crow.utils.files import get_full_path

try:
    from IPython import embed
except ImportError:
    import code

    def embed():
        vars = globals()
        vars.update(locals())
        shell = code.InteractiveConsole(vars)
        shell.interact()
# the above is not needed - the only purpose is to have a nice way
# of pausing the code execution in this simple example
from time import time
from opcua import Client  # import the Client class
from opcua import ua
import sounddevice as sd
import speech_recognition as sr
import nltk
import numpy as np
import json
import os
import sys
import pyaudio
from pydub import AudioSegment
from pydub.playback import play
import io

from speech_vr.encoder.MorphCategory import POS
from speech_vr.encoder.ParsedText import ParsedText, TaggedText, ParseTreeNode, TaggedToken  # noqa
from speech_vr.encoder.Tag import Tag
from speech_vr.encoder.NLProcessor import RawNLParser
from google_speech import Speech, SpeechSegment, PreloaderThread, PRELOADER_THREAD_COUNT
try:
    from inputimeout import inputimeout, TimeoutOccurred
    INPUTIMEOUT_IMPORT_SUCCESS = True
except ImportError:
    INPUTIMEOUT_IMPORT_SUCCESS = False
import argparse
from pkg_resources import resource_filename
from unidecode import unidecode
import traceback
from concurrent.futures import TimeoutError as ConcurrentTimeoutError
import re
from threading import Thread
import subprocess
from select import select

from pynput.keyboard import Key, Listener, KeyCode


def trace_exception(limit=2):
    exc_type, exc_value, exc_traceback = sys.exc_info()
    traceback.print_exception(exc_type, exc_value, exc_traceback, limit=limit, file=sys.stdout)

def kbhit():
    dr,dw,de = select([sys.stdin], [], [], 0)
    return dr != []

class SpeechThread(Thread):

    def __init__(self, text, lang="en-US", sox_effects=(), *args, **kwargs):
        super().__init__(daemon=True, *args, **kwargs)
        self.speech = SpeechCustom(text, lang)
        self.sox_effects = sox_effects
        self.should_terminate = False
        self.terminated = False

    def run(self):
        self.speech.play(self.sox_effects)
        while True:
            if all([segment.process.poll() is not None for segment in self.speech.segments]):
                break
            if self.should_terminate:
                for segment in self.speech.segments:
                    segment.terminate()
                break
        self.terminated = True

    def terminate(self):
        self.should_terminate = True


class SpeechSegmentCustom(SpeechSegment):

    def __init__(self, text, lang, segment_num, segment_count=None):
        super().__init__(text, lang, segment_num, segment_count=None)
        self.process = None

    def play(self, sox_effects=()):
        """ Play the segment. """
        audio_data = self.getAudioData()
        # cmd = ["sox", "-q", "-t", "mp3", "-"]
        # if sys.platform.startswith("win32"):
        #     cmd.extend(("-t", "waveaudio"))
        #     # cmd.extend(("-d", "trim", "0.1", "reverse", "trim", "0.07", "reverse"))  # "trim", "0.25", "-0.1"
        # cmd.extend(sox_effects)
        # self.process = subprocess.Popen(cmd,
        #                                 stdin=subprocess.PIPE,
        #                                 stdout=subprocess.DEVNULL)
        # # p.communicate(input=audio_data)
        # self.process.stdin.write(audio_data)
        # self.process.stdin.flush()
        # self.process.stdin.close()
        # if self.process.wait() != 0:
        #     raise RuntimeError()

        data = audio_data
        song = AudioSegment.from_file(io.BytesIO(data), format="mp3")
        play(song)


    def terminate(self):
        self.process.terminate()


class SpeechCustom(Speech):

    def __next__(self):
        """ Get a speech segment, splitting text by taking into account spaces, punctuation, and maximum segment size. """
        if self.text == "-":
            if sys.stdin.isatty():
                return
            while True:
                new_line = sys.stdin.readline()
                if not new_line:
                    return
                segments = __class__.splitText(new_line)
                for segment_num, segment in enumerate(segments):
                    yield SpeechSegmentCustom(segment, self.lang, segment_num, len(segments))
        else:
            segments = __class__.splitText(self.text)
            for segment_num, segment in enumerate(segments):
                yield SpeechSegmentCustom(segment, self.lang, segment_num, len(segments))

    def play(self, sox_effects=()):
        """ Play a speech. """
        # Build the segments
        preloader_threads = []
        self.segments = list(self)
        # start preloader thread(s)
        preloader_threads = [PreloaderThread(name="PreloaderThread-%u" % (i)) for i in range(PRELOADER_THREAD_COUNT)]
        for preloader_thread in preloader_threads:
            preloader_thread.segments = self.segments
            preloader_thread.start()

        # destroy preloader threads
        for preloader_thread in preloader_threads:
            preloader_thread.join()

        # play segments
        for segment in self.segments:
            segment.play(sox_effects)


class SpeechProcessor():
    WRONG_INPUT_STATE = 0
    SEND_ERRORS = False  # whether to send error to OPC server or just show them in console
    USE_ACTIVE_STATE = False
    STRIP_ACCENTS = True  # whether to remove accents before comparing recognized text and possible choices
    ALLOW_INTERRUPTIONS = False  # whether the user can interupt TTS playback

    MICROPHONE_WAIT_TIMEOUT = 5  # time to wait for any non-zero audio from mic
    LISTENING_START_TIMEOUT = 5  # time to start speaking after recognition is ran
    PHRASE_TIMEOUT = 4  # maximum length of phrase before listening is cut off
    CALIBRATION_TIME = 1  # time to spend calibrating the microphone bgr energy levels

    def __init__(self, serverAdress, gain: int = 1.0):
        # instantiate client object; make sure address and port are correct
        self.DEBUG_mode = False
        self.client = Client(serverAdress)
        self.shouldProcessInput = True
        self.LANG = 'en'
        self.recognizer = sr.Recognizer()
        self.fast_recognizer = sr.Recognizer()
        self.fast_recognizer.non_speaking_duration = 0.05
        self.fast_recognizer.pause_threshold = 0.15
        self.recText = RawNLParser(language=self.LANG)
        self.lastRecognitionFailed = False
        self.repeatChoices = False

        self.last_key = ''
        self.keys_pressed = set()
        if self.DEBUG_mode:
            self.keys_pressed = 's'

        self.ADD_OBJECTS = 1

        self.kb_listener = Listener(on_press=self.on_press)
        self.kb_listener.start()

        self.play_sound("start_speech")
        """ SoX Effects
        http://sox.sourceforge.net/sox.html#EFFECTS
        gain
            -n normalize the audio to 0dB
            -b balance the audio, try to prevent clipping
        dither
            adds noise to mask low sampling rate
        vol
            changes volume:
                above 1 -> increase volume
                below 1 -> decrease volume
        pad
            adds silence to the begining (first argument) and
            the end (second argument) of the audio
            - attempt to mask sound cut-off but extends the audio duration
        """
        self.sox_effects = ("gain", "-n", "-b", "vol", str(gain), "pad", "0", "0.5", "dither", "-a")

        SpeechCustom.MAX_SEGMENT_SIZE = 300
        Speech.MAX_SEGMENT_SIZE = 300
        self.hint_directive_re = re.compile(r"<([b-])>\s*$")

        rospy.init_node('process_sentence_node', anonymous=False)

        logging.config.fileConfig(get_full_path('config', 'logging.ini'))

        self._getdatsrv = rospy.ServiceProxy('/db_interface_node/getDatabase', GetDatabase)

        req = GetDatabaseRequest()
        req.write = False
        res = self._getdatsrv.call(req)

        self.sub = rospy.Subscriber('/nl_input', SentenceProgram, queue_size=10, callback=self.callback)
        self.pub = rospy.Publisher('/sentence_output', SentenceProgram, queue_size=10)
        self._addsrv = rospy.ServiceProxy('/db_interface_node/addObject', AddObject)
        self._senddatsrv = rospy.ServiceProxy('/db_interface_node/sendDatabase', SendDatabase)

        self.db_api = DatabaseAPI()
        self.db = self.db_api.get_db()
        self.ontoC = self.db.change_onto(res.path)

        # obj = self.db.onto.search(type = db.onto.Cube)
        # obj2 = self.ontoC.search(type = db.onto.Cube)

        # obj3 = self.db.onto.search(type = db.onto.Cube)
        # obj4 = self.ontoC.search(type = db.onto.Cube)

        # path = '/home/algernon/ros_melodic_ws/base_ws/src/crow_nlp/scripts/saved_updated_onto.owl'

        self.UngroundedID = 1
        self.GroundedID = 1

    def on_press(self, key):
        self.last_key = key

        if isinstance(key, KeyCode):
            self.keys_pressed.add(key.char)

        print('{0} pressed'.format(
            key))

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

    def _send_state(self, state_num):
        """
        Changes state to the specified number and sets "request" to True.
        """
        dt = ua.DataValue(ua.Variant(state_num, ua.VariantType.Int16))
        self.request_state_num.set_value(dt)
        self.request.set_value(True)

    def __extract_directive(self, string):
        result = self.hint_directive_re.split(string)
        if len(result) > 1:
            return result[0], result[1]
        else:
            return result[0], ""

    def init(self, configure_deivce=False):
        self.load_files()

        print("Audio input devices on this system:\n\tindex\tname")
        default_device_idx = sd.default.device[0]
        #default_device_idx = 12
        device_list = sr.Microphone.list_microphone_names()
        for index, name in enumerate(device_list):
            if index == default_device_idx:
                d = ">\t"
                add = "(CURRENT)"
            else:
                d = "\t"
                add = ""
            print(f"{d}{index}:\t\t{name} {add}")

        self.microphone_index = 12#default_device_idx
        if type(configure_deivce) is bool and configure_deivce:
            if INPUTIMEOUT_IMPORT_SUCCESS:
                timout = 10
                try:
                    self.microphone_index = int(inputimeout(
                        prompt=f"Select device to use within {timout} seconds (enter to select the default device)\nDefault device '{default_device_idx}: {device_list[default_device_idx]}': ", timeout=timout) or default_device_idx)
                except TimeoutOccurred:
                    print("Selecting the default device.")
            else:
                self.microphone_index = int(input(
                    prompt=f"Select device to use (enter to select the default device)\nDefault device '{default_device_idx}: {device_list[default_device_idx]}': ") or default_device_idx)
            if self.microphone_index != default_device_idx:
                print(f"Selected device with index {self.microphone_index}: {device_list[self.microphone_index]}.")
            else:
                print("Selected the default device.")
        elif type(configure_deivce) is int:
            self.microphone_index = configure_deivce
            print(f"Selected device with index {self.microphone_index}: {device_list[self.microphone_index]} from command line.")

        self.stream_device = [self.microphone_index, sd.default.device[1]]

        print("Calibrating microphone. Make sure it is turned on and that no one is speaking.")
        self._notbreak = True
        with sr.Microphone(device_index=self.microphone_index) as source:
            # wait for a second to let the recognizer adjust the
            # energy threshold based on the surrounding noise level
            self.recognizer.adjust_for_ambient_noise(source, duration=self.CALIBRATION_TIME)
            self.fast_recognizer.energy_threshold = self.recognizer.energy_threshold
            # self.recognizer.dynamic_energy_threshold = True
        print("Calibration done.")
        self.print_message("Ready.")

    def say(self, req, say=True, screen=True):
        """
        produce a text via different output methods.

        self.say('tell me more.')

        :param req: text to output.
        :return:
        """

        if screen:
            print(req)
        if say:
            # make request to google to get synthesis
            tts = gtts.gTTS(req,lang=self.LANG)
            # save the audio file
            tts.save("say.mp3")
            # play the audio file
            playsound("say.mp3")
            # proc = Popen(['spd-say', req])
            # os.system(f'spd-say "{req}"')

    def listen(self):
        success = False
        recog_text_original = ""
        try:
            with sr.Microphone(device_index=self.microphone_index) as source:
                print("You may say Something")
                # self.play_sound("start_speech")

                # self.play_message("Řekněte, jakou možnost si přejete ..")
                # listens for the user's input
                audio = self.recognizer.listen(source, timeout=self.LISTENING_START_TIMEOUT,
                                               phrase_time_limit=self.PHRASE_TIMEOUT)

            # self.play_sound("end_speech")
            print("speech heard, processing...")
            # for testing purposes, we're just using the default API key
            # to use another API key, use `self.recognizer.recognize_google(audio, key="GOOGLE_SPEECH_RECOGNITION_API_KEY")`
            # speech recognition
            recog_text_original = self.recognizer.recognize_google(audio, language=self.LANG)
            print(recog_text_original)
            success = True
        except sr.UnknownValueError as e:
            # self.print_message(self.create_response("did_not_understand", locals()), block=True)
            print(f"Did not understand: {e}")
        except sr.WaitTimeoutError:
            # self.print_message(self.create_response("no_speech", locals()))
            print(f"No speech")
        else:
            # self.print_message(self.create_response("speech_recognition_result", locals()))
            print(f"I recognized text: {recog_text_original}")

        if success:
            return recog_text_original
        else:
            return ""

    def text_preprocessing(self, text_raw):

        # if success or (self.ALLOW_INTERRUPTIONS and recog_text_original):
        # processing recognized text
        recog_text = self.recText.replace_synonyms(text_raw)
        # self.sentences.append(recog_text)
        # rospy.sleep(1.0)
        msg = SentenceProgram()
        # msg.header.stamp = rospy.Time.now()
        msg.data.append(recog_text)
        # print("Publishing: {}".format(msg.data))
        # self.pub.publish(msg)
        # self.whole = False
        # self.sentences = []
        # print("Recognized text after synonyms substitution: ", recog_text)
        # load possible variants for next states
        # process recorded text using parser, tagging
        #tagged_text = self.get_tagged_text(recog_text)
        # TODO: more intelligent way of catching this

        return msg

    def add_object(self, type, x, y, z, id=None, color=None):
        req = AddObjectRequest()
        req.obj_class = type
        req.x = x
        req.y = y
        req.z = z
        if id:
            req.id = id
        if color:
            req.color = color
        req.action.action = DBOperation.ADD
        res = self._addsrv.call(req)
        assert isinstance(res, AddObjectResponse)
        return

    def get_database(self, write):
        req = GetDatabaseRequest()
        req.write = write
        res = self._getdatsrv.call(req)

        self.ontoC = self.db.change_onto(res.path)
        obj = self.ontoC.search(type=self.ontoC.Cube)
        if write:
            self.ontoC = self.db.onto.__enter__()
        return

    def send_database(self):
        req = SendDatabaseRequest()
        self.ontoC.__exit__()
        path = os.path.dirname(os.path.abspath(__file__)) + '/saved_updated_onto.owl'
        self.ontoC.save(path)
        # rospy.sleep(1.0)

        req.path = path
        res = self._senddatsrv.call(req)
        print(res.received.msg)
        print(res.received.success)
        # self.db.onto.__exit__()
        return

    def process_sentences(self, input_sentences):
        # populate the workspace
        #TODO should be replaced by input from a realsense
        if self.ADD_OBJECTS == 1:
            self.add_object('Glue', x=0.2, y=0.4, z=0, color='black')
            self.add_object('Panel', x=0.2, y=0.1, z=0, color = 'red')
            self.add_object('Panel', x=0.2, y=-0.2, z=0, color = 'blue')

            self.add_object('Cube', x=0.1, y=0.3, z=0, id='0', color='red')
            self.add_object('Cube', x=0.1, y=0.2, z=0, id='1', color='red')
            self.add_object('Cube', x=0.2, y=0.2, z=0, id='2', color='green')
            self.ADD_OBJECTS = 0
        # self.add_object(onto.Screwdriver, x=0.1, y=0.3, z=0, id='0', color='red')
        # self.add_object('Screwdriver', x=0.1, y=0.2, z=0, id='1', color='red')
        # self.add_object('Screwdriver', x=0.2, y=0.2, z=0, id='2', color='green')
        # get current database after adding all objects

        self.get_database(write=False)

        # just to see if objects were added to the database
        obj = self.db.onto.search(type=db.onto.Cube)
        print(obj)
        obj = self.db.onto.search(type=db.onto.Glue)
        print(obj)
        obj = self.db.onto.search(type=db.onto.Panel)
        print(obj)

        for input_sentence in input_sentences:

            self.get_database(write=True)

            #self.ontoC = self.db.onto.__enter__()
            nl_processor = NLProcessor(language = self.LANG)
            program_template = nl_processor.process_text(input_sentence)
            # get current database state for writing an ungrounded and currently grounded program to be executed

            print()
            print("Program Template")
            print("--------")
            print(program_template)

            if self.db_api.get_state() == State.DEFAULT:
                # self.save_unground_program(program_template)
                self.send_database()
                self.get_database(write=True)

               # self.ontoC = self.db.onto.__enter__()
                robot_program = self.run_program(program_template)
                if self.db_api.get_state() == State.DEFAULT:
                    self.save_grounded_program(robot_program)
                    self.send_database()
                elif self.db_api.get_state() != State.LEARN_FROM_INSTRUCTIONS:
                    self.db_api.set_state(State.DEFAULT)
                    self.send_database()

            elif self.db_api.get_state() == State.LEARN_FROM_INSTRUCTIONS:
                self.save_new_template(program_template)
                self.send_database()

        # print list of programs
        self.get_database(write=False)
        all_custom_templates = self.db_api.get_custom_templates()
        for custom_template in all_custom_templates:
            print(custom_template.name[1:])
        all_programs = self.db.onto.search(type=self.db.onto.RobotProgram)
        path = os.path.dirname(os.path.abspath(__file__)) + '/saved_updated_onto.owl'
        for program in all_programs:
            print(program.name)
        return

    def save_grounded_program(self, ground_program):
        # save to database and when database with the program sent,
        # we sent a message that the ground program was written to
        # database and new database sent
        # TODO search ontology for last program id
        # TODO link the corresponding ungrounded program in grounded one or vice versa
        # TODO add parameter time to the added program
        name = 'grounded_' + str(self.GroundedID)
        self.GroundedID = self.GroundedID + 1
        self.db_api.save_program(ground_program, name)

        return

    def save_new_template(self, program_template):
        self.db_api.add_custom_template(program_template)
        self.db_api.set_state(State.DEFAULT)
        return

    def run_program(self, program_template):
        program_runner = ProgramRunner(language = self.LANG)
        robot_program = program_runner.evaluate(program_template)
        print()
        print("Grounded Program")
        print("--------")
        print(robot_program)
        return robot_program


    def callback(self, data):
        input_sentences = data.data
        self.process_sentences(input_sentences)
        return


    def run(self):
        """
        The main function of this function is to hold the code execution.
        """
        # embed()  # this just pauses the code be entering IPython console, type "quit()" to quit or press ctrl+D

        # say "Hello World"
        # self.play_message("Slyšela jsem, že chceš postavit koně, teď se bude stavět.")

        while True:
            # key = self._readInput()
            # print(self._decode_states(self.next_state_possibilities.get_value()))
            # print(self.actual_state_number.get_value())
            # print(self.mic_active.get_value())

            # if key == "q":
            if 'q' in self.keys_pressed:
                self.print_message("User requested termination.")
                break
            elif 's' in self.keys_pressed:
                print('detecting robot programs using ontology.')
                if not self.DEBUG_mode:
                    self.say(self.guidance_file[self.LANG]["start_template"])
                    self.say(self.guidance_file[self.LANG]["start_specify"])

                    text = self.listen()
                if self.DEBUG_mode:
                    if self.LANG == 'cs':
                        # sentence_program.data = ["Polož kostka na pozici 3 3"]
                        text = "Nalep bod sem a polož kostku sem."
                        #text = "Ukliď červenou kostku."
                    if self.LANG == 'en':
                        text = "Glue a point here and Put cube to position 0 0"
                        text = "Tidy up red cube."
                    self.say(self.guidance_file[self.LANG]["debug_text"] + text)
                if text == "":
                    self.play_message(self.create_response("did_not_understand", locals()), block=True)
                    self.print_message(self.create_response("no_speech", locals()))
                    continue
                    #raise NotImplementedError('What should happen when no text was recognized??')
                else:
                    sentence_program = self.text_preprocessing(text)
                    print(sentence_program)
                    self.process_sentences(sentence_program.data)
                    self.keys_pressed = set()
                    print('processing complete')

            elif self.shouldProcessInput:
                # set mic_active back to False -> we started processing the input
                self.mic_active.set_value(False)

                # retrieve current and possible states
                # list of next state possibilities
                possible_states = self._decode_states(self.next_state_possibilities.get_value())
                current_state = self.actual_state_number.get_value()

                # print(f"We are currently in state {current_state_text}.") #" and the possible choices are {text_possible_states}.")
                # self.play_message(f"Nyní jsme ve stavu {current_state_text}.") # možnosti k výběru jsou {text_possible_states}.")

                next_state = self.processInput(current_state, possible_states)  # the chosen next state
                if next_state is not None and next_state > 0:
                    try:
                        # send state change request
                        self._send_state(next_state)
                    except Exception:
                        print("There was an error while sending the chosen next state. The error message was:")
                        trace_exception()
                    else:
                        self.shouldProcessInput = False
                        self.lastRecognitionFailed = False
                else:
                    self.lastRecognitionFailed = True
                    # recognition failed, attempt again on the next cycle
                    continue

    def processInput(self, current_state, possible_states):
        """
        This function should listen for audio and process the speech.
        It will be called only if user might enter an input.
        """
        next_state_ID = -1
        recog_text_original = ""

        [variants, possible_states, current_state_text] = self.get_variants(current_state, possible_states)
        if current_state_text == "":  # no state hint, assuming passthrough state
            print("no state hint. Going to next state.")
            return possible_states[0]

        current_state_text, hint_directive = self.__extract_directive(current_state_text)
        if hint_directive == "-":  # a passthrough state
            self.play_message(self.create_response(
                "current_state", locals()), display=True)
            # automatically advance to the next state
            return possible_states[0]

        if not self.lastRecognitionFailed or self.repeatChoices:
            start_message = self.create_response("current_state", locals())
            if not self.lastRecognitionFailed:
                if len(possible_states) > 1:
                    start_message += self.create_response("which_choice", locals())
                elif hint_directive == "b":
                    start_message += self.create_response("yes_or_no", locals())
            if self.ALLOW_INTERRUPTIONS:
                interrupted_text = self.play_message_and_listen(start_message, display=True)
                recog_text_original = interrupted_text + " "
            else:
                self.play_message(start_message, display=True)
            self.repeatChoices = False

        # breaking when above zero
        # if not self.block_until_sound():
        #     return -1

        # speech to text from microphone
        success = False
        try:
            with sr.Microphone(device_index=self.microphone_index) as source:
                print("You may say Something")
                self.play_sound("start_speech")

                # self.play_message("Řekněte, jakou možnost si přejete ..")
                # listens for the user's input
                audio = self.recognizer.listen(source, timeout=self.LISTENING_START_TIMEOUT, phrase_time_limit=self.PHRASE_TIMEOUT)

            self.play_sound("end_speech")
            print("speech heard, processing...")
            # for testing purposes, we're just using the default API key
            # to use another API key, use `self.recognizer.recognize_google(audio, key="GOOGLE_SPEECH_RECOGNITION_API_KEY")`
            # speech recognition
            recog_text_original += self.recognizer.recognize_google(audio, language=self.LANG)
            # recog_text_original = 'velky velikost'
        except sr.UnknownValueError:
            self.play_message(self.create_response("did_not_understand", locals()), block=True)
        except sr.WaitTimeoutError:
            self.print_message(self.create_response("no_speech", locals()))
        else:
            self.print_message(self.create_response("speech_recognition_result", locals()))
            success = True

        if success or (self.ALLOW_INTERRUPTIONS and recog_text_original):
            # processing recognized text
            recog_text = self.recText.replace_synonyms(recog_text_original)
            # print("Recognized text after synonyms substitution: ", recog_text)
            # load possible variants for next states
            # process recorded text using parser, tagging
            tagged_text = self.get_tagged_text(recog_text)
            # TODO: more intelligent way of catching this
            #if ["moznosti", "možnost", "volby", "volba", "výběr"] in tagged_text:
            if "CHOICE_option" in tagged_text:
                self.repeatChoices = True
                return

            if len(variants) == 0:
                # TODO if nothing matching...
                print("No choices for this state. Is this an error?")
                return

            # select variant for the given actual state from possible next states
            if hint_directive == "b":
                #yes, no = [choice in tagged_text for choice in [
                #    ["ano", "muze", "jo", "preji", "chci","jasně","jasne","podej","dej","můžeš", "chtěla","můžete","muzete","muzes","podejte"],
                #    ["ne", "nemuze","nechci","nedavej"]
                #]]
                #TODO yes is checked before no. What to do if both words?
                yes = "YES_option" in tagged_text
                no = "NO_option" in tagged_text
                # yes, no = [choice in tagged_text for choice in [
                #     self.synonyms_file[self.LANG]["YES_option"],
                #     self.synonyms_file[self.LANG]["NO_option"]
                # ]]
                if yes and no:
                    self.play_message(self.create_response("makes_no_sense", locals()))
                    return
                elif yes:
                    self.print_message("")
                    return possible_states[0]
                elif no:
                    self.play_message(self.create_response("no_cannot_proceed", locals()))
                    return

            variantID, next_state = self.select_variant(tagged_text, variants)
            if variantID > -1:
                selected_variant = variants[variantID]
                next_state_ID = possible_states[variantID]
                self.print_message(self.create_response("selected_state_full", locals()))
                self.play_message(self.create_response("selected_state", locals()))
            else:
                print(f"Tagged text: {tagged_text}")
                print(f"Variants: {variants}")
                self.play_message(self.create_response("unknown_choice", locals()))

        return next_state_ID

    def create_response(self, response_id, local_variables={}, language=None):
        if "self" not in local_variables:
            local_variables["self"] = self
        response = ""
        try:
            response = self.responses[self.LANG if language is None else language][response_id].format(**local_variables)
        except KeyError as e:
            print(f"Variable missing from locals: {e}")
            print(f"Locals: {local_variables}")
        return response

    def print_message(self, message, raw=True):
        print(message)
        if self.SEND_ERRORS:
            self.response_string.set_value(message)

    def play_message(self, text, display=False, block=True):
        """Plays a message. The message is transformed into speech using TTS.
        Optionally, the message can be output to the console.

        Parameters
        ----------
        text : str
            The text to be played as speech.
        display : bool, optional
            If true, the text is also displayed in the console (to remove the need for extra function calls).
            The *print_message* function is used for that. By default False
        """
        if display:
            self.print_message(text)
        self.current_speech = SpeechThread(text, self.LANG, self.sox_effects)
        self.current_speech.start()
        if block:
            # self.block_until_sound(threshold=1, timeout=20)
            # if self.ALLOW_INTERRUPTIONS:
            #     self.current_speech.terminate()
            # else:
            #     self.current_speech.join()
            self.current_speech.join()

    def play_message_and_listen(self, text, display=False):
        self.play_message(text, display=display, block=False)
        while not self.current_speech.terminated:
            recog_text = ""
            try:
                with sr.Microphone(device_index=self.microphone_index) as source:
                    audio = self.fast_recognizer.listen(source, phrase_time_limit=1)
                    recog_text = self.fast_recognizer.recognize_google(audio, language=self.LANG)
            except Exception:
                continue
            else:
                if recog_text:
                    return recog_text
        return ""

    def play_sound(self, sound_name):
        #cmd = ["sox", "-q", "-t", "wav", resource_filename("speech_vr", f"sounds/{sound_name}.wav")]
        playsound(resource_filename("speech_vr", f"sounds/{sound_name}.wav"))
        # cmd = ["mplayer", resource_filename("speech_vr", f"sounds/{sound_name}.wav")]
        # if sys.platform.startswith("win32"):
        #     cmd.extend(("-t", "waveaudio"))
        # #cmd.extend(("gain", "-n", "vol", "1"))
        # subprocess.Popen(cmd)

    def block_until_sound(self, threshold=0.1, timeout=MICROPHONE_WAIT_TIMEOUT):
        # waiting for stream of data from microphone
        self._notbreak = True
        start_time = time()
        with sd.Stream(callback=lambda indata, outdata, frames, time, status, threshold=threshold: self.__print_sound(indata, outdata, frames, time, status, threshold), device=self.stream_device):
            while self._notbreak:
                # sd.sleep(0.1)
                if time() - start_time > timeout:
                    self.print_message(self.create_response("no_audio", locals()))
                    return False
        return True

    def __print_sound(self, indata, outdata, frames, time, status, threshold):
        volume_norm = np.linalg.norm(indata) * 10
        # print(int(volume_norm))
        if volume_norm > threshold:
            self._notbreak = False

    def get_variants(self, current_state, possible_states=[]):
        if self.USE_ACTIVE_STATE:
            # only based on the active_state
            print("Variant list: ", self.next_states_list)
            print(f"Current state ID: {current_state}")
            possible_states = self.next_states_list[self.LANG][str(current_state)]
            print('Loaded possible next states:', possible_states)

        # or based on the next_state_possibilities
        variants = [self.variants_list[self.LANG][str(option)] for option in possible_states]

        # text description of the current state
        current_state_text = self.state_hints_list[self.LANG][str(current_state)] if str(current_state) in self.state_hints_list[self.LANG] else ""
        return variants, possible_states, current_state_text

    def load_files(self):
        # root_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)))
        # variants_file = os.path.join(root_dir, "utils", "state_description_next.json")
        variants_file = resource_filename("speech_vr", os.path.join("utils", "state_description.json"))
        with open(variants_file, "r", encoding="utf-8") as f:
            self.variants_list = json.load(f)

        if self.USE_ACTIVE_STATE:
            next_states_file = resource_filename("speech_vr", os.path.join("utils", "next_states.json"))
            with open(next_states_file, "r", encoding="utf-8") as f:
                self.next_states_list = json.load(f)

        state_hints_file = resource_filename("speech_vr", os.path.join("utils", "state_hints.json"))
        with open(state_hints_file, "r", encoding="utf-8") as f:
            self.state_hints_list = json.load(f)

        state_hints_file = resource_filename("speech_vr", os.path.join("utils", "responses.json"))
        with open(state_hints_file, "r", encoding="utf-8") as f:
            self.responses = json.load(f)

        synonyms_file = resource_filename("speech_vr", os.path.join("utils", "synonyms.json"))
        with open(synonyms_file, "r", encoding="utf-8") as f:
            self.synonyms_file = json.load(f)

        guidance_file = resource_filename("speech_vr", os.path.join("utils", "guidance_dialogue.json"))
        with open(guidance_file, "r", encoding="utf-8") as f:
            self.guidance_file = json.load(f)

    def select_variant(self, tagged_text, variants):
        sel_variant_ID = -1
        selected_variant = None
        # if len(variants) == 1:
        #     sel_variant_ID = 0
        #     selected_variant = variants[0]
        #     print('only one next state')
        # else:
        if self.STRIP_ACCENTS:
            variants = [unidecode(var) for var in variants]
        for variant_ID in range(len(variants)):
            # TODO handle multiple variants detection
            # TODO exchange for variants[variant_ID] in tagged_text - string works same, if list, if at least one in the list, gives back true
            if tagged_text.contains_pos_token(variants[variant_ID], '*'):
                print('Detected variant:', variants[variant_ID])
                selected_variant = variants[variant_ID]
                sel_variant_ID = variant_ID
        return sel_variant_ID, selected_variant

    def get_tagged_text(self, recog_text):
        tagged_text = TaggedText()
        tagged_text.tokens = []
        tagged_text.tags = []
        tokens = nltk.word_tokenize(recog_text)
        # print(tokens)
        for pair in nltk.pos_tag(tokens):
            tag = Tag()
            tag.pos = POS(value=pair[1])
            tagged_text.tokens.append(unidecode(pair[0]) if self.STRIP_ACCENTS else pair[0])
            tagged_text.tags.append(tag)
        return tagged_text

    def datachange_notification(self, node, val, data):
        """
        Subscription handler for "mic_active"
        """
        if not val:
            return  # the "mic_active" is False -> do nothing
        print("The mic is active!")
        # self.client.connect()  # <- wtf point
        self.shouldProcessInput = True

    def event_notification(self, event):
        print("New event received: ", event)

    def _decode_states(self, next_state_values):
        if type(next_state_values[0]) is int:
            return next_state_values
        else:
            return [v for v in [int.from_bytes(b[:2], byteorder="little") for b in [
                ns.Body for ns in next_state_values]] if v > 0]

    def _readInput(self, timeout=0.1):
        # start_time = time()
        # sys.stdout.write('%s(%s):' % (caption, default))
        inp = ''
        if self.last_key == '':
            return ''
        if not isinstance(self.last_key, KeyCode):
            return ''
        # while True:
        # if msvcrt.kbhit():
        # if kbhit():
        # if self.last_key != '':
            # char = msvcrt.getche()
        if self.last_key.char == 'q':
            inp = self.last_key.char
        if self.last_key.char == 's':
            inp = self.last_key.char

        #         char = getch.getche()
        #         if ord(char) == 13:  # enter_key
        #             break
        #         elif ord(char) >= 32:  # space_char
        #             inp = chr(ord(char))
        #             break
        #     if len(inp) == 0 and (time() - start_time) > timeout:
        #         break
        # # print('')  # needed to move to next line
        self.last_key = ''

        if len(inp) > 0:
            return inp
        else:
            return ""

    def disconnect(self):
        try:
            self.sub.unsubscribe(self.handle)  # cancel subscription
            self.sub.delete()
        except AttributeError:
            pass  # not subscribed, yet
        except Exception:
            print("Error trying to unsubscribe from the mic_active variable!")
        self.client.disconnect()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("host", nargs="?", default="local", help="OPC host server address. Use 'local' for a local host or 'last' for last used (non-local) address.")
    parser.add_argument("--configure_device", "-d", nargs="?", default=-2, const=-1)  # default == False value, const == True value, some Natural int == device index
    parser.add_argument("--gain", "-g", default=None, type=float)
    parser.add_argument("--allow_interrupt", "-i", action="store_true", default=False)
    args = parser.parse_args()

    with open(resource_filename("speech_vr", "config.json"), "r") as f:
        config = json.load(f)

    storeAddress = False
    if not args.host or args.host == "last":
        address = config["last_address"]
    elif args.host == "local":
        address = config["local_address"]
    else:
        address = args.host
        storeAddress = True

    if args.gain is not None:
        config["gain"] = args.gain
        with open(resource_filename("speech_vr", "config.json"), "w") as f:
            json.dump(config, f)
        gain = args.gain
    else:
        gain = config["gain"]

    SpeechProcessor.ALLOW_INTERRUPTIONS = args.allow_interrupt
    sp = SpeechProcessor(address, gain=gain)

    print(f"Connecting to {address}.")
    try:
        sp.connect()  # try to connect and setup necessary variables
    except ua.uaerrors.BadNoMatch:
        print("The requested variable does not exist on the OPC server!")
        trace_exception()
        sp.disconnect()
        sys.exit(-2)
    except KeyboardInterrupt:
        print("User requested interruption.")
        sp.disconnect()
        sys.exit(0)
    except Exception:
        print("Connecting to the server failed!")
        trace_exception()
        try:
            sp.disconnect()
        except Exception:
            pass
        sys.exit(-2)
    else:
        print('Connected to OPC server.')
        if storeAddress:
            config["last_address"] = address
            with open(resource_filename("speech_vr", "config.json"), "w") as f:
                json.dump(config, f)

    cdev = int(args.configure_device)
    if cdev < 0:
        cdev = True if cdev == -1 else False
    try:
        sp.init(configure_deivce=cdev)
    except AttributeError as ae:
        print(f"Sound initialization and calibration failed due to:\n'{ae}'\nThis is likely caused by missing PyAudio or incompatible version of it.\nExiting.")
        try:
            sp.disconnect()
        except Exception:
            pass
        sys.exit(-2)
    except KeyboardInterrupt:
        print("User requested interruption.")
        sp.disconnect()
        sys.exit(0)
    except Exception:
        trace_exception()
        try:
            sp.disconnect()
        except Exception:
            pass
        sys.exit(-2)

    try:
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
            sp.disconnect()  # always disconnect at the end
        except Exception:
            trace_exception()
