#!/usr/bin/env python3
"""
Copyright (c) 2019 CIIRC, CTU in Prague
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.

@author: Zdenek Kasner
"""

from nlp_crow.database.Database import Database
from nlp_crow.database.DatabaseAPI import DatabaseAPI
from nlp_crow.database.Ontology import Template
from nlp_crow.modules.LocationDetector import LocationDetector

import owlready2 as ow

from nlp_crow.modules.LocationGrounder import LocationGrounder
from nlp_crow.modules.ObjectDetector import ObjectDetector
from nlp_crow.modules.ObjectGrounder import ObjectGrounder
from nlp_crow.structures.tagging.TaggedText import TaggedText
from nlp_crow.modules.UserInputManager import UserInputManager

import logging

db = Database()
db_api = DatabaseAPI()

# with db.onto as onto:
class TidyTask(Template):
    namespace = db.onto
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.register_parameter(name="object_to_put", value=db.onto.Object)
        self.logger = logging.getLogger(__name__)
        self.ui = UserInputManager()

        self.templ_det = self.ui.load_file('templates_detection.json')

    def match(self, tagged_text : TaggedText, language = 'en') -> None:
        od = ObjectDetector(language = language)

        # TODO: temporary solution: detect the "put subject" in 3 words after the "put" verb. Should be improved with better grammar parsing.
        try:
            put_index = tagged_text.indices_of(self.templ_det[language]['tidy'])[0]
            tagged_text_cut = tagged_text.cut(put_index + 1, put_index + 4)

            self.object_to_put = od.detect_object(tagged_text_cut)
        except:
            self.logger.debug(f"Put index not detected.")

    def evaluate(self, language = 'en') -> None:
        # check if the object to be put down is in the workspace
        self.lang = language
        og = ObjectGrounder(language = self.lang)
        self.object_to_put = og.ground_object(obj_placeholder=self.object_to_put)


        # if not obj:
        #     self.location = None
        #     self.logger.warning("No object to be put down.")
        #     return

        # # TODO this should generally be done only if the task is successful (or outside of this scope completely)
        # obj._is_picked = False
        # obj._can_be_picked = True
        # db_api.update_object_position(obj.id, x=self.location.x, y=self.location.y)

