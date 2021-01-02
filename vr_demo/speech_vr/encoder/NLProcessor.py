import os
import re
import json

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