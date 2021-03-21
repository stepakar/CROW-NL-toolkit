#!/usr/bin/env python
"""
Copyright (c) 2020 CIIRC CTU in Prague
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.

@author: Karla Stepanova, karla.stepanova@cvut.cz
"""
import unittest
from pprint import pprint as pp
from nlp_crow.database.DatabaseAPI import DatabaseAPI
from nlp_crow.processing.NLProcessor import NLProcessor
from nlp_crow.modules.UserInputManager import UserInputManager


class UserInputTest(unittest.TestCase):

    def setUp(self):
        LANG = 'en'
        self.ui = UserInputManager(LANG)
        self.orig_say = self.ui.say
        self.text = []
        self.say_flag = []
        self.screen_flag = []
        def say(text : str, say=True, screen=True):
            self.text.append(text)
            self.say_flag.append(say)
            self.screen_flag.append(screen)
            self.orig_say(text)
        self.ui.say = say




    def test_query_state_inform_en(self):
        # LANG = 'en'
        # # process a NL instruction
        # ui = UserInputManager(LANG)
        state_description = "Basic state"
        query_type = 'Inform'
        mode = 'Silent'

        ret = self.ui.query_state(state_description, query_type, mode)

        print(self.text)
        print(self.say_flag)
        print(self.screen_flag)
        self.assertTrue("state" in self.text[0].lower())
        self.assertTrue(state_description.lower() in self.text[0].lower())
        self.assertIsNone(ret, msg='return value is {}, but should be None.'.format(ret))
        self.assertFalse(self.say_flag[0], msg='Silent flag was set, but speech was spoken')
        self.assertTrue(self.screen_flag[0], msg='Silent flag was set, but speech not displayed')
        # self.assertTrue(isinstance(ret, str), msg='return value is not a string')

    def test_query_state_inform_cs(self):
        LANG = 'cs'
        # # process a NL instruction
        # ui = UserInputManager(LANG)
        self.ui.lang = LANG
        state_description = "základní stav"
        query_type = 'Inform'
        mode = 'Silent'

        ret = self.ui.query_state(state_description, query_type, mode)

        print(self.text)
        print(self.say_flag)
        print(self.screen_flag)
#        self.assertTrue(self.text[0] == "Nyní jsme ve stavu " + state_description)
        self.assertTrue("stavu" in self.text[0].lower())
        self.assertTrue(state_description.lower() in self.text[0].lower())
        self.assertIsNone(ret, msg='return value is {}, but should be None.'.format(ret))
        self.assertFalse(self.say_flag[0], msg='Silent flag was set, but speech was spoken')
        self.assertTrue(self.screen_flag[0], msg='Silent flag was set, but speech not displayed')


    def join_text(self, text_list):
        text = ""
        for s in text_list:
            text += s + ' '

        return text

    def test_query_state_select_en(self):
        # LANG = 'en'
        # # process a NL instruction
        # ui = UserInputManager(LANG)
        state_description = "Selection state"
        query_variants = ['bear','mouse','horse']
        query_type = 'Select'
        mode = 'Silent'

        ret = self.ui.query_state(state_description, query_type, query_variants, mode)

        print(self.text)
        print(self.say_flag)
        print(self.screen_flag)

        text = self.join_text(self.text)
        print(text)

        self.assertTrue("state" in text.lower())
        self.assertTrue(state_description.lower() in text.lower())
        self.assertTrue("bear, mouse or horse." in text.lower())
        self.assertTrue("which option" in text.lower())
        self.assertIsNone(ret, msg='return value is {}, but should be None.'.format(ret))
        self.assertFalse(self.say_flag[0], msg='Silent flag was set, but speech was spoken')
        self.assertTrue(self.screen_flag[0], msg='Silent flag was set, but speech not displayed')
        # self.assertTrue(isinstance(ret, str), msg='return value is not a string')

    def test_query_state_select_cs(self):
        LANG = 'cs'
        # # process a NL instruction
        # ui = UserInputManager(LANG)
        self.ui.lang = LANG
        state_description = "Výběr stavu"
        query_variants = ['medvěd', 'myška', 'kůň']
        query_type = 'Select'
        mode = 'Silent'

        ret = self.ui.query_state(state_description, query_type, query_variants, mode)

        print(self.text)
        print(self.say_flag)
        print(self.screen_flag)

        text = self.join_text(self.text)
        print(text)

        self.assertTrue("stavu" in text.lower())
        self.assertTrue(state_description.lower() in text.lower())
        self.assertTrue("medvěd, myška nebo kůň." in text.lower())
        self.assertTrue("jakou možnost" in text.lower())
        self.assertIsNone(ret, msg='return value is {}, but should be None.'.format(ret))
        self.assertFalse(self.say_flag[0], msg='Silent flag was set, but speech was spoken')
        self.assertTrue(self.screen_flag[0], msg='Silent flag was set, but speech not displayed')
        # self.assertTrue(isinstance(ret, str), msg='return value is not a string')

    def test_query_state_yes_no_en(self):
        # LANG = 'en'
        # # process a NL instruction
        # ui = UserInputManager(LANG)
        state_description = "Decision state"
        query_variants = ['attach the rope']
        query_type = 'Yes_or_no'
        mode = 'Silent'

        ret = self.ui.query_state(state_description, query_type, query_variants, mode)

        print(self.text)
        print(self.say_flag)
        print(self.screen_flag)
        self.assertTrue("state" in self.text[0].lower())
        self.assertTrue(state_description.lower() in self.text[0].lower())
        self.assertTrue("want to attach" in self.text[0].lower())
        self.assertIsNone(ret, msg='return value is {}, but should be None.'.format(ret))
        self.assertFalse(self.say_flag[0], msg='Silent flag was set, but speech was spoken')
        self.assertTrue(self.screen_flag[0], msg='Silent flag was set, but speech not displayed')
        # self.assertTrue(isinstance(ret, str), msg='return value is not a string')

    def test_query_state_yes_no_cs(self):
        LANG = 'cs'
        # # process a NL instruction
        # ui = UserInputManager(LANG)
        self.ui.lang = LANG
        state_description = "Rozhodovací stav"
        query_variants = ['připojit lano']
        query_type = 'Yes_or_no'
        mode = 'Silent'

        ret = self.ui.query_state(state_description, query_type, query_variants, mode)

        print(self.text)
        print(self.say_flag)
        print(self.screen_flag)
        self.assertTrue("stavu" in self.text[0].lower())
        self.assertTrue(state_description.lower() in self.text[0].lower())
        self.assertTrue("připojit lano" in self.text[0].lower())
        self.assertIsNone(ret, msg='return value is {}, but should be None.'.format(ret))
        self.assertFalse(self.say_flag[0], msg='Silent flag was set, but speech was spoken')
        self.assertTrue(self.screen_flag[0], msg='Silent flag was set, but speech not displayed')
        # self.assertTrue(isinstance(ret, str), msg='return value is not a string')


if __name__ == '__main__':
    unittest.main()