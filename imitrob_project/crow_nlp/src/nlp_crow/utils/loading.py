from pkg_resources import resource_filename
import json
import os

class Loading:
    #enables to load files and other variables
    def load_files(self):
        # root_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)))
        # variants_file = os.path.join(root_dir, "utils", "state_description_next.json")
        variants_file = resource_filename("crow_nlp", os.path.join("utils", "state_description.json"))
        with open(variants_file, "r", encoding="utf-8") as f:
            self.variants_list = json.load(f)

        if self.USE_ACTIVE_STATE:
            next_states_file = resource_filename("crow_nlp", os.path.join("utils", "next_states.json"))
            with open(next_states_file, "r", encoding="utf-8") as f:
                self.next_states_list = json.load(f)

        state_hints_file = resource_filename("crow_nlp", os.path.join("utils", "state_hints.json"))
        with open(state_hints_file, "r", encoding="utf-8") as f:
            self.state_hints_list = json.load(f)

        state_hints_file = resource_filename("crow_nlp", os.path.join("utils", "responses.json"))
        with open(state_hints_file, "r", encoding="utf-8") as f:
            self.responses = json.load(f)