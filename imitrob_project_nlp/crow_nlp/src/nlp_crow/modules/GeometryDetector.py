#!/usr/bin/env python
"""
Copyright (c) 2019 CIIRC, CTU in Prague
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.

@author: Zdenek Kasner
"""
import logging

from nlp_crow.database.Database import Database
from nlp_crow.modules.CrowModule import CrowModule
from nlp_crow.structures.tagging.TaggedText import TaggedText
from nlp_crow.modules.UserInputManager import UserInputManager

db = Database()

# with db.onto as onto:
class GeometryDetector(CrowModule):
    """
    Detects geometry in text - e.g. point, line, etc.
    """
    namespace = db.onto

    def __init__(self, language = 'en'):
        self.lang = language
        self.logger = logging.getLogger(__name__)
        self.ui = UserInputManager()

        self.templ_det = self.ui.load_file('templates_detection.json')

    def detect_geometry(self, tagged_text : TaggedText):
        go = None

        if tagged_text.contains_text(self.templ_det[self.lang]['point']):
            go = db.onto.GeometricObject("point")

        if go:
            self.logger.debug(f"Geometry detected for \"{tagged_text.get_text()}\": {go}")

        return go
