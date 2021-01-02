

"""
ROS action server example
"""
import os
from subprocess import Popen

import numpy as np
import rospy
import actionlib

import speech_recognition as sr
import sounddevice as sd

import gtts
from playsound import playsound


# def execute(goal):
#     curve = np.array([[c.x, c.y, c.z] for c in goal.curve])
#     normals = np.array([[n.x, n.y, n.z] for n in goal.normals])
#     pause = rospy.Duration(0.005)
#     n_points = curve.shape[0]
#     success = True
#     rospy.loginfo("Starting curve execution.")
#     for i, (c, n) in enumerate(zip(curve, normals)):
#         if server.is_preempt_requested():
#             server.set_preempted()
#             success = False
#             break
#
#         rospy.loginfo("> c: {}\t n: {} ({:.2f}%)".format(c, n, float(i) / n_points * 100))
#         server.publish_feedback(CurveExecutionFeedback(progress=float(i) / n_points * 100))
#         rospy.sleep(pause)
#     if success:
#         # create result/response message
#         server.set_succeeded(CurveExecutionResult(True))
#         rospy.loginfo('Action successfully completed')
#     else:
#         server.set_aborted(CurveExecutionResult(False))
#         rospy.loginfo('Whoops')

# TODO: which RAWNLParser should be imported???
from crow_nlp.msg import SentenceProgram, SpeechUserInteractionAction, SpeechUserInteractionActionGoal, nlp_enum, \
    SpeechUserInteractionActionResult, SpeechUserInteractionActionFeedback, SpeechUserInteractionGoal, \
    SpeechUserInteractionFeedback, SpeechUserInteractionResult

from nlp_crow.speech_recognition.SpeechProcessor import SpeechProcessor
from vr_demo_factorio.speech_vr.encoder.NLProcessor import RawNLParser


class SpeechUI(object):
    WRONG_INPUT_STATE = 0
    SEND_ERRORS = False  # whether to send error to OPC server or just show them in console
    USE_ACTIVE_STATE = False
    STRIP_ACCENTS = True  # whether to remove accents before comparing recognized text and possible choices
    ALLOW_INTERRUPTIONS = False  # whether the user can interupt TTS playback

    MICROPHONE_WAIT_TIMEOUT = 5  # time to wait for any non-zero audio from mic
    LISTENING_START_TIMEOUT = 5  # time to start speaking after recognition is ran
    PHRASE_TIMEOUT = 4  # maximum length of phrase before listening is cut off
    CALIBRATION_TIME = 1  # time to spend calibrating the microphone bgr energy levels


    def __init__(self):
        self.server = actionlib.SimpleActionServer('SpeechUI', SpeechUserInteractionAction, self.interaction_dispatcher, False)

        self.sp = SpeechProcessor()
        # parser.add_argument("--configure_device", "-d", nargs="?", default=-2,
        #                     const=-1)
        self.sp.init(configure_deivce=False)
        # self.setup_mic()



        self.server.start()


    def interaction_dispatcher(self, req):
        assert isinstance(req, SpeechUserInteractionGoal)
        res = SpeechUserInteractionResult()
        feedback = SpeechUserInteractionFeedback()
        feedback.state = nlp_enum.RECEIVED
        self.server.publish_feedback(feedback)

        # TODO: implement preemtion checks


        if req.request_type == nlp_enum.DETECT_PROGRAM:
            feedback.state = nlp_enum.SPEAKING
            self.server.publish_feedback(feedback)
            self.say('Please, specify actions!')

            if self.server.is_preempt_requested():
                self.server.set_preempted(text='preempted')
                return

            feedback.state = nlp_enum.LISTENING
            self.server.publish_feedback(feedback)
            text = self.listen()
            if text == "":
                raise NotImplementedError('What should happen when no text was recognized??')

            if self.server.is_preempt_requested():
                self.server.set_preempted(text='preempted')
                return

            feedback.state = nlp_enum.PROCESSING
            self.server.publish_feedback(feedback)
            sentence_program = self.text_preprocessing(text)
            res.sentences = sentence_program.data

            self.server.set_succeeded(res)
            rospy.loginfo('Action successfully completed')
            return
        elif req.request_type == nlp_enum.SYSTEM_REQUEST:
            feedback.state = nlp_enum.SPEAKING
            self.server.publish_feedback(feedback)

            self.sp.processInput(current_state=re)

            self.say(req.user_msg)

            if self.server.is_preempt_requested():
                self.server.set_preempted(text='preempted')
                return

            feedback.state = nlp_enum.LISTENING
            self.server.publish_feedback(feedback)
            text = self.listen()
            if text == "":
                raise NotImplementedError('What should happen when no text was recognized??')

            if self.server.is_preempt_requested():
                self.server.set_preempted(text='preempted')
                return

            feedback.state = nlp_enum.PROCESSING
            self.server.publish_feedback(feedback)

            # State change detection from language
            res.next_state = 80

            self.server.set_succeeded(res)
            rospy.loginfo('Action successfully completed')
            return

        self.server.set_aborted(text=f'no valid request received: {req.request_type}')

    def say(self, req, say=True, screen=True, log=True, action_feedback=True):
        """
        produce a text via different output methods.

        self.say('tell me more.')

        :param req: text to output.
        :return:
        """



        proc = None
        if action_feedback:
            feedback = SpeechUserInteractionFeedback()
            feedback.state = nlp_enum.SPEAKING
            feedback.msg = req
            self.server.publish_feedback(feedback)
        if screen:
            print(req)
        if log:
            rospy.loginfo(f'Saying: {req}')
        if say:
            # make request to google to get synthesis
            tts = gtts.gTTS(req)
            # save the audio file
            tts.save("say.mp3")
            # play the audio file
            playsound("say.mp3")
            # proc = Popen(['spd-say', req])
            # os.system(f'spd-say "{req}"')


        # if not proc is None:
        #     proc.wait(timeout=5.0)

    def listen(self):
        success = False
        recog_text_original = ""
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
            recog_text_original = self.recognizer.recognize_google(audio, language=self.LANG)
            print(recog_text_original)
            success = True
        except sr.UnknownValueError as e:
            #self.print_message(self.create_response("did_not_understand", locals()), block=True)
            print(f"Did not understand: {e}")
        except sr.WaitTimeoutError:
            #self.print_message(self.create_response("no_speech", locals()))
            print(f"No speech")
        else:
            #self.print_message(self.create_response("speech_recognition_result", locals()))
            print(f"I recognized text: {recog_text_original}")

        if success:
            return recog_text_original
        else:
            return ""

    def text_preprocessing(self, text_raw):

        # if success or (self.ALLOW_INTERRUPTIONS and recog_text_original):
        # processing recognized text
        recog_text = self.recText.replace_synonyms(text_raw)
        self.sentences.append(recog_text)
        rospy.sleep(1.0)
        msg = SentenceProgram()
        msg.header.stamp = rospy.Time.now()
        msg.data = self.sentences
        # print("Publishing: {}".format(msg.data))
        # self.pub.publish(msg)
        self.whole = False
        self.sentences = []
        # print("Recognized text after synonyms substitution: ", recog_text)
        # load possible variants for next states
        # process recorded text using parser, tagging
        #tagged_text = self.get_tagged_text(recog_text)
        # TODO: more intelligent way of catching this

        return msg



    def setup_mic(self):
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



if __name__ == '__main__':
    rospy.init_node('dummy_controller')

    sui = SpeechUI()
    # Similarly to service, advertise the action server
    # server = actionlib.SimpleActionServer('curve_executor', CurveExecutionAction, execute, False)
    # server.start()
    rospy.spin()


