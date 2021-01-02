#!/usr/bin/env python3
"""
Copyright (c) 2019 CIIRC CTU in Prague
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.

@author: Gabriela Sejnova, Karla Stepanova
@mail: karla.stepanova@cvut.cz
"""
import os

from crow_nlp.msg import SentenceProgram
import rospy
import speech_recognition as sr
import sounddevice as sd
from asr.msg import *
import time
import re
import json
from pkg_resources import resource_filename
from google_speech import Speech, SpeechSegment, PreloaderThread, PRELOADER_THREAD_COUNT
from threading import Thread
import traceback

def trace_exception(limit=2):
    exc_type, exc_value, exc_traceback = sys.exc_info()
    traceback.print_exception(exc_type, exc_value, exc_traceback, limit=limit, file=sys.stdout)


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
        cmd = ["sox", "-q", "-t", "mp3", "-"]
        if sys.platform.startswith("win32"):
            cmd.extend(("-t", "waveaudio"))
            # cmd.extend(("-d", "trim", "0.1", "reverse", "trim", "0.07", "reverse"))  # "trim", "0.25", "-0.1"
        cmd.extend(sox_effects)
        self.process = subprocess.Popen(cmd,
                                        stdin=subprocess.PIPE,
                                        stdout=subprocess.DEVNULL)
        # p.communicate(input=audio_data)
        self.process.stdin.write(audio_data)
        self.process.stdin.flush()
        self.process.stdin.close()
        # if self.process.wait() != 0:
        #     raise RuntimeError()

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


class RawNLParser:
    def __init__(self, feature=None, language="cs", splitter="scenario"):
        """
        This class takes the raw text input and outputs the filled gherkin template in NL. It also updates
        the RobotPrograms.json with detected actions of selected subject
        :param feature: the actor we are talking about in the scenarios, i.e. robot R3
                (is filled when there is only "robot" without specification)
        :param language: language of the NL text (en/cs)
        :param subj_of_interest: subject of whom we collect actions and save them to RobotPrograms.json
        :param splitter: word which separates scenarios in NL text
        """
        self.lang = language
        self.split_word = splitter
        #self.subj_of_interest = c.SUBJ_OF_INTEREST
        self.actor_uni = feature
        #self.cleaner = NLTempMatcher(lang=self.lang, nmm=nmm, feature=feature)
        #self.finetune = Finetuner(self.lang, nmm)
        self.text_raw = None
        self.robot_actions = []
        self.text_lines_normalized = []
        self.sentences = []
        #self.keywords = [g.GIVEN, g.WHEN, g.THEN]
        #if self.lang == "cs":
            #self.keywords = [g.GIVEN_CS, g.WHEN_CS, g.THEN_CS]


    def replace_synonyms(self, command):
        """place words in command with synonyms defined in synonym_file"""
        root_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir)
        synonyms_filename = os.path.join(
            root_dir, "utils", "synonyms.json")
        with open(synonyms_filename, encoding="utf-8") as f:
            synonyms = json.load(f)
        for key in synonyms[self.lang]:
            for phrase in synonyms[self.lang][key]:
                if phrase in command.lower():
                    src_str = re.compile(r'\b{}\b'.format(phrase), re.IGNORECASE)
                    command = src_str.sub(key, command)
        command = self.strip_extra_spaces(re.sub(r"\stodelete", "", command))
        number_words = re.findall(r'\b({}|{})\b\s*[0-9]'.format("číslo", "number"), command, re.IGNORECASE)
        for x in number_words:
            command = re.sub(r'\b{}\b'.format(x), "", command, re.IGNORECASE)
        return command

    def strip_extra_spaces(self, text):
        stripped_spaces = re.sub(' +', ' ', text)
        stripped_text = stripped_spaces.strip()
        return stripped_text

class nl_input_node():
    # listens to /averaged_markers from object_detection package and in parallel to button presses. When there is an approval
    # to collect cubes (key a -> y), robot picks up the currently detected cubes and places them to
    #### Subscriber reads speech transcripts published by /asr_output and once there is silence again, it publishes them as list
    WRONG_INPUT_STATE = 0
    SEND_ERRORS = False  # whether to send error to OPC server or just show them in console
    USE_ACTIVE_STATE = False
    STRIP_ACCENTS = True  # whether to remove accents before comparing recognized text and possible choices
    ALLOW_INTERRUPTIONS = False  # whether the user can interupt TTS playback

    MICROPHONE_WAIT_TIMEOUT = 5  # time to wait for any non-zero audio from mic
    LISTENING_START_TIMEOUT = 5  # time to start speaking after recognition is ran
    PHRASE_TIMEOUT = 4  # maximum length of phrase before listening is cut off
    CALIBRATION_TIME = 1  # time to spend calibrating the microphone bgr energy levels

    def __init__(self, gain: int = 1.0):
        rospy.init_node('nl_input_node', anonymous=False)
        self.sentences = []
        self.whole = False
        #self.sub = rospy.Subscriber('/asr_output', asr_output, queue_size=1, callback=self.callback)
        self.pub = rospy.Publisher('/nl_input', SentenceProgram, queue_size=10)
        self.shouldProcessInput = True
        self.LANG = 'cs'
        self.recognizer = sr.Recognizer()
        self.fast_recognizer = sr.Recognizer()
        self.fast_recognizer.non_speaking_duration = 0.05
        self.fast_recognizer.pause_threshold = 0.15
        self.recText = RawNLParser()
        self.lastRecognitionFailed = False
        self.repeatChoices = False
        # sd.default.device = 'digital output'

        # self.play_sound("start_speech")
        # """ SoX Effects
        # http://sox.sourceforge.net/sox.html#EFFECTS
        # gain
        #     -n normalize the audio to 0dB
        #     -b balance the audio, try to prevent clipping
        # dither
        #     adds noise to mask low sampling rate
        # vol
        #     changes volume:
        #         above 1 -> increase volume
        #         below 1 -> decrease volume
        # pad
        #     adds silence to the begining (first argument) and
        #     the end (second argument) of the audio
        #     - attempt to mask sound cut-off but extends the audio duration
        # """
        # self.sox_effects = ("gain", "-n", "-b", "vol", str(gain), "pad", "0", "0.5", "dither", "-a")

        next_state_ID = -1
        recog_text_original = ""
        self.load_files()

        print("Audio input devices on this system:\n\tindex\tname")
        default_device_idx = sd.default.device[0]
        device_list = sr.Microphone.list_microphone_names()
        for index, name in enumerate(device_list):
            if index == default_device_idx:
                d = ">\t"
                add = "(CURRENT)"
            else:
                d = "\t"
                add = ""
            print(f"{d}{index}:\t\t{name} {add}")

        self.microphone_index = default_device_idx

        try:
            with sr.Microphone(device_index=self.microphone_index) as source:
                print("You may say Something")
                #self.play_sound("start_speech")

                # self.play_message("Řekněte, jakou možnost si přejete ..")
                # listens for the user's input
                audio = self.recognizer.listen(source, timeout=self.LISTENING_START_TIMEOUT,
                                               phrase_time_limit=self.PHRASE_TIMEOUT)

            #self.play_sound("end_speech")
            print("speech heard, processing...")
            # for testing purposes, we're just using the default API key
            # to use another API key, use `self.recognizer.recognize_google(audio, key="GOOGLE_SPEECH_RECOGNITION_API_KEY")`
            # speech recognition
            recog_text_original += self.recognizer.recognize_google(audio, language=self.LANG)
            print(recog_text_original)
        except sr.UnknownValueError:
            #self.print_message(self.create_response("did_not_understand", locals()), block=True)
            print(f"Did not understand{recog_text_original}")
        except sr.WaitTimeoutError:
            #self.print_message(self.create_response("no_speech", locals()))
            print(f"No speech")
        else:
            #self.print_message(self.create_response("speech_recognition_result", locals()))
            print(f"I recognized text: {recog_text_original}")
            success = True

        if success or (self.ALLOW_INTERRUPTIONS and recog_text_original):
            # processing recognized text
            recog_text = self.recText.replace_synonyms(recog_text_original)
            self.sentences.append(recog_text)
            rospy.sleep(1.0)
            msg = SentenceProgram()
            msg.header.stamp = rospy.Time.now()
            msg.data = self.sentences
            print("Publishing: {}".format(msg.data))
            self.pub.publish(msg)
            self.whole = False
            self.sentences = []
            # print("Recognized text after synonyms substitution: ", recog_text)
            # load possible variants for next states
            # process recorded text using parser, tagging
            #tagged_text = self.get_tagged_text(recog_text)
            # TODO: more intelligent way of catching this

    def load_files(self):
        # root_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)))
        # variants_file = os.path.join(root_dir, "utils", "state_description_next.json")
        root_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir)
        variants_file = os.path.join(
            root_dir, "utils", "state_description.json")
        # variants_file = resource_filename("crow_nlp", os.path.join("utils", "state_description.json"))
        with open(variants_file, "r", encoding="utf-8") as f:
            self.variants_list = json.load(f)

        if self.USE_ACTIVE_STATE:
            next_states_file = os.path.join(
                root_dir, "utils", "next_states.json")
            # next_states_file = resource_filename("crow_nlp", os.path.join("utils", "next_states.json"))
            with open(next_states_file, "r", encoding="utf-8") as f:
                self.next_states_list = json.load(f)

        state_hints_file = os.path.join(
            root_dir, "utils", "state_hints.json")
        # state_hints_file = resource_filename("crow_nlp", os.path.join("utils", "state_hints.json"))
        with open(state_hints_file, "r", encoding="utf-8") as f:
            self.state_hints_list = json.load(f)

        responses_file = os.path.join(
            root_dir, "utils", "responses.json")
        # responses_file = resource_filename("crow_nlp", os.path.join("utils", "responses.json"))
        with open(responses_file, "r", encoding="utf-8") as f:
            self.responses = json.load(f)

    def print_message(self, message, raw=True):
        print(message)
        if self.SEND_ERRORS:
            self.response_string.set_value(message)

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


if __name__ == '__main__':
        nl_input_node()
        # spin() simply keeps python from exiting until this node is stopped
