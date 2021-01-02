#!/usr/bin/env python3
"""
Copyright (c) 2019 CIIRC, CTU in Prague
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.

@author: Zdenek Kasner
"""
from nlp_crow.database.Database import Database
from nlp_crow.database.Ontology import Template


import owlready2 as ow
import nlp_crow.modules.LocationGrounder as LocationGrounder

from nlp_crow.structures.tagging.TaggedText import TaggedText
from nlp_crow.modules.UserInputManager import UserInputManager

db = Database()

# with db.onto as onto:
class ApplyGlueTask(Template):
    namespace = db.onto
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.register_parameter(name="size", value=db.onto.GeometricObject)
        self.register_parameter(name="position", value=db.onto.Location)

        self.ui = UserInputManager()

        self.templ_det = self.ui.load_file('templates_detection.json')

    def match(self, tagged_text : TaggedText, language = 'en') -> None:
        import nlp_crow.modules.GeometryDetector as GeometryDetector
        import nlp_crow.modules.LocationDetector as LocationDetector

        gd = GeometryDetector.GeometryDetector(language = language)
        ld = LocationDetector.LocationDetector(language = language)

        self.size = gd.detect_geometry(tagged_text)
        self.position = ld.detect_location(tagged_text)

    def evaluate(self,language = 'en') -> None:
        if db.onto.RelativeLocation in self.position.is_instance_of:
            lg = LocationGrounder.LocationGrounder(language = self.lang)

            self.position = lg.ground_location(loc=self.position)


