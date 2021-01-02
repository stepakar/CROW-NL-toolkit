#!/usr/bin/env python
"""
Copyright (c) 2019 CIIRC, CTU in Prague
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.

@author: Zdenek Kasner
"""
import json
import os
from typing import List

from pkg_resources import resource_filename

from nlp_crow.database.DatabaseAPI import DatabaseAPI
from nlp_crow.modules.CrowModule import CrowModule
from nlp_crow.structures.tagging.MorphCategory import POS
from nlp_crow.structures.tagging.ParsedText import TaggedText
from nlp_crow.structures.tagging.Tag import Tag
from nlp_crow.templates.TemplateFactory import TemplateType as tt
from nlp_crow.modules.UserInputManager import UserInputManager

db_api = DatabaseAPI()
db = db_api.get_db()
# onto = db_api.get_onto()

# with onto:
class TemplateDetector(CrowModule):
    """
    First part of the NL pipeline. Preliminary template detection in the text.
    """
    namespace = db.onto
    def __init__(self,language = 'en'):
        self.lang = language
        self.ui = UserInputManager(language = language)

    def detect_templates(self, tagged_text : TaggedText) -> List[tt]:
        """
        Tries to guess which template should be used to represent robot instructions for the chunk of
        text. Can detect more templates, but only the first one which matches will be used later.

        Parameters
        ----------
        tagged_text  a tagged text in which the template should be detected

        Returns
        -------
        a list of guessed templates for the tagged text sorted by their probability
        """

        self.templ_det = self.ui.load_file('templates_detection.json')
        self.guidance_file = self.ui.load_file('guidance_dialogue.json')

        templates = []
        detect_fns = [self.detect_pick,
                          self.detect_apply_glue,
                          self.detect_put,
                          self.detect_tidy,
                          self.detect_learn,
                          self.detect_tower,
                          self.detect_demonstration_list,
                          self.detect_define_area]

        # try to find custom templates (compound actions) first
        custom_templates = self.detect_custom_templates(tagged_text)

        if custom_templates:
            templates += custom_templates

        # add detected basic templates (actions)
        for detect_fn in detect_fns:
            res = detect_fn(tagged_text)

            if res:
                templates += res

        return templates


    def detect_custom_templates(self, tagged_text : TaggedText):
        """
        Retrieves learned custom templates (compound actions) from the database
        and tries to detect them in the text.

        Parameters
        ----------
        tagged_text  a tagged text in which a custom template should be detected

        Returns
        -------
        a list of custom templates detected in the text, an empty list if no template is detected
        """
        all_custom_templates = db_api.get_custom_templates()

        custom_templates = []

        for custom_template in all_custom_templates:
            custom_template_name = custom_template.name[1:]

            if custom_template_name.lower() in tagged_text.get_text().lower():
                custom_templates.append(custom_template)
        return custom_templates


    def detect_pick(self, tagged_text : TaggedText) -> List[tt]:
        """
        Detector for PickTask
        """
        if tagged_text.contains_pos_token(self.templ_det[self.lang]['take'], "VB") or tagged_text.contains_pos_token(self.templ_det[self.lang]['pick'], "VB"):
            self.ui.say(self.guidance_file[self.lang]["template_match"]+self.templ_det[self.lang]['pick'])

            #if tagged_text.contains_pos_token("take", "VB") or \
        #        tagged_text.contains_pos_token("pick", "VB"):
            return [tt.PICK_TASK]

    def detect_apply_glue(self, tagged_text : TaggedText) -> List[tt]:
        """
        Detector for ApplyGlueTask
        """
        if tagged_text.contains_pos_token(self.templ_det[self.lang]['glue'], "VB") or tagged_text.contains_pos_token(self.templ_det[self.lang]['glue'], "NNS")  or tagged_text.contains_pos_token(self.templ_det[self.lang]['glue'], "NNP"):
            self.ui.say(self.guidance_file[self.lang]["template_match"] + self.templ_det[self.lang]['glue'])
            return [tt.APPLY_GLUE]

    def detect_learn(self, tagged_text : TaggedText) -> List[tt]:
        """
        Detector for LearnNewTask
        """
        if tagged_text.contains_text("learn") and tagged_text.contains_text("new task"):
            self.ui.say(self.guidance_file[self.lang]["template_match"] + self.templ_det[self.lang]['learn_new_task'])
            return [tt.LEARN_NEW_TASK]

    def detect_put(self, tagged_text: TaggedText) -> List[tt]:
        """
        Detector for PutTask
        """
        #if tagged_text.contains_text("put"):
        if tagged_text.contains_text(self.templ_det[self.lang]['put']):
            self.ui.say(self.guidance_file[self.lang]["template_match"]+ self.templ_det[self.lang]['put'])
            return [tt.PUT_TASK]

    def detect_tidy(self, tagged_text: TaggedText) -> List[tt]:
        """
        Detector for PutTask
        """
        #if tagged_text.contains_text("put"):
        if tagged_text.contains_text(self.templ_det[self.lang]['tidy']):
            self.ui.say(self.guidance_file[self.lang]["template_match"]+ self.templ_det[self.lang]['tidy'])
            return [tt.TIDY_TASK]

    def detect_tower(self, tagged_text: TaggedText) -> List[tt]:
        """
        Detector for LearnTowerFromDemonstration
        """
        if tagged_text.contains_text("learn") and tagged_text.contains_text("tower"):
            self.ui.say(self.guidance_file[self.lang]["template_match"]+ self.templ_det[self.lang]['learn_new_tower'])
            return [tt.LEARN_TOWER]


    def detect_demonstration_list(self, tagged_text: TaggedText) -> List[tt]:
        """
        Detector for DemonstrationList
        """
        if tagged_text.contains_text("show") and tagged_text.contains_text("demonstration"):
            self.ui.say(self.guidance_file[self.lang]["template_match"] + self.templ_det[self.lang]['demonstration'])
            return [tt.DEMONSTRATION_LIST]


    def detect_define_area(self, tagged_text: TaggedText) -> List[tt]:
        """
        Detector for DefineArea
        """
        if tagged_text.contains_text("define") and tagged_text.contains_text("area"):
            self.ui.say(self.guidance_file[self.lang]["template_match"] + self.templ_det[self.lang]['define_area'])
            return [tt.DEFINE_AREA]
