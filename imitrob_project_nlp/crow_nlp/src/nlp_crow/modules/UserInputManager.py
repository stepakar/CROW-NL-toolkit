#!/usr/bin/env python
"""
Copyright (c) 2020 CIIRC, CTU in Prague
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.

@author: Zdenek Kasner, Karla Stepanova
"""
import json
import os
from functools import wraps
from typing import List, Any

from pkg_resources import resource_filename

from nlp_crow.modules.CrowModule import CrowModule
from pprint import pprint as pp

import logging
import gtts
from playsound import playsound


def repeat(message="", count=5):
    """
    A decorator which allows to simply repeat a request for user input.
    After each failed attempt, a message is displayed to the user. Usage:

    @repeat(message="Please type a valid number.")
    def ask_for_number(self):
        (...)
        if success:
            return number
        return None

    Parameters
    ----------
    message message which is displayed after each failed attempt
    count   maximum number of attempts
    """
    def decorate(func):
        @wraps(func)
        def wrapper(self):
            for i in range(count):
                ret = func(self)
                if ret is not None:
                    return ret
                UserInputManager.say(message)
            logging.debug("Maximum number of attempts reached")

        return wrapper
    return decorate

class UserInputManager(CrowModule):
    """
    Provides methods for human-robot interaction in terms of command line input/output.
    In the future, the methods can be extended to work with interactive screen etc.
    """
    def __init__(self, language = 'en'):
        self.lang = language
        self.templ_det = self.load_file('templates_detection.json')
        self.guidance_file = self.load_file('guidance_dialogue.json')

    def ask_to_select_from_class_objects(self, objs : List[Any]) -> Any:
        """
        In case multiple objects of the same class (and certain properties) have been found
        in the workspace, we ask the user to select one of them.

        Parameters
        ----------
        objs  list of objects to select from

        Returns
        -------
        the selected object
        """
        l = len(objs)

        # dummy "human-robot interaction"
        self.say(self.guidance_file[self.lang]["found_object"]+f"{l}" + self.guidance_file[self.lang]["object_type"]+ f"{objs[0].__class__}" + self.guidance_file[self.lang]["in_workspace"])
        self.say(self.guidance_file[self.lang]["select_object"] +
                      f"{l - 1}:\n\t")

        self.show(dict(enumerate(objs)))

        i = self.enter_count_user()

        return objs[i]

    @repeat(message="Ok, lets try it once more...")
    def ask_for_name(self, entity) -> str:
        """
        Ask the user for the name of an entity.

        Returns
        -------
        the name of the entity
        """
        self.say(f"What is the name of the {entity}?")

        name = self.enter_identifier_user()

        self.say(f"The name of the action is {name}, is it ok?")

        if self.confirm_user():
            return name

    # def ask_for_action_name(self) -> str:
    #     self.say("What is the name of the action?")
    #
    #     name = self.enter_identifier_user()
    #
    #     self.say(f"The name of the action is {name}, is it ok?")
    #
    #     if self.confirm_user():
    #         return name


    def ask_for_input(self) -> str:
        """
        Ask the user for a text input from the keyboard.

        Returns
        -------
        the input from the keyboard
        """
        sentence = input()

        return sentence


    @repeat(message="Enter a valid identifier")
    def enter_identifier_user(self) -> str:
        """
        Ask the user for a valid identifier (variable name, action name, etc.).

        Returns
        -------
        a valid identifier or None if no attempt is valid
        """
        id = input()

        if self.check_identifier(id):
            return id

    @repeat(message="Enter a valid number")
    def enter_count_user(self) -> int:
        """
        Ask the user for a valid number.

        Returns
        -------
        a valid number or None if no attempt is valid
        """
        #TODO uncomment it
        self.DEFAULT_SELECTION = 1
        if self.DEFAULT_SELECTION:
            i = "0"
            self.say(self.guidance_file[self.lang]["default_option"])
        else:
            i = input()

        if i.isdigit():
            return int(i)
    @repeat(message="Sorry, I didn't understand you. Answer either y/yes or n/no.")
    def confirm_user(self) -> bool:
        """
        Ask user for confirmation.

        Returns
        -------
        True if the user confirms, False if the user does not confirm,
        None if no attempt is valid.
        """
        conf = input().lower()

        if conf in ["yes", "y"]:
            return True

        elif conf in ["no", "n"]:
            return False


    def check_identifier(self, id : str) -> bool:
        """
        Checks if the string is a valid identifier.

        Parameters
        ----------
        id  the string to be checked

        Returns
        -------
        True if the identifier is valid, False otherwise
        """
        # TODO add some checks
        return True

    #@staticmethod
    def say(self, text : str, say=True, screen=True) -> None:
        """
        Emulates a robotic way of "saying" things to the user.
        Can be extended to work with real speech synthesis.

        Parameters
        ----------
        text   the text to be said by the robot
        """
        #print(text)

        if screen:
            print(text)
        if say:
            # make request to google to get synthesis
            tts = gtts.gTTS(text, lang = self.lang)
            # save the audio file
            tts.save("say.mp3")
            # play the audio file
            playsound("say.mp3")
            # proc = Popen(['spd-say', req])
            # os.system(f'spd-say "{req}"')

    @staticmethod
    def show(obj : Any) -> None:
        """
        Emulates a robotic way of "showing" things to the user.
        Can be extended to work with an interactive screen.

        Parameters
        ----------
        obj   an object to be shown
        """
        # use python Pretty-Print
        pp(obj)

    def load_file(self,file, package = 'speech_vr',folder = 'utils'):
        templates_file = resource_filename(package, os.path.join(folder, file))
        with open(templates_file, "r", encoding="utf-8") as f:
            self.templ_det = json.load(f)
        return self.templ_det