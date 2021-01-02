#!/usr/bin/env python
"""
Copyright (c) 2019 CIIRC, CTU in Prague
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.

@author: Zdenek Kasner
"""
from nlp_crow.modules.CrowModule import CrowModule
from nlp_crow.structures.tagging.TaggedText import TaggedText
from nlp_crow.modules.UserInputManager import UserInputManager


class ColorDetector(CrowModule):
    """
    Detects colors in text.
    """
    def __init__(self,language = 'en'):
        self.colors = ["red", "blue", "black", "green"]
        self.lang = language
        self.ui = UserInputManager(language = self.lang)
        self.templ_det = self.ui.load_file('templates_detection.json')

    def detect_color(self, text : TaggedText):
        """
        A simple method for detecting a color in text.
        Yes, there is some space for improvement.

        Parameters
        ----------
        text  an input text
        """
        for color in self.colors:
            try:
                color_lang = self.templ_det[self.lang][color]
                if color_lang in text.get_text():
                    return color
            except:
                pass

        return None