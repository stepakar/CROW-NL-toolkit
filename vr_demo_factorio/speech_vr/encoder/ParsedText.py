#!/usr/bin/env python
"""
Copyright (c) 2019 CIIRC, CTU in Prague
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.

@author: Zdenek Kasner
"""

from speech_vr.encoder.TaggedText import TaggedText, TaggedToken


# with db.onto as onto:
class ParsedText():
    """
    The text parsed into a tree based on the sentence structure and the grammar used for parsing.
    """
    def get_tagged_text(self) -> TaggedText:
        """
        Returns the tagged tokens without tree structure.
        """
        return self.parse_tree.flatten()

    def __str__(self):
        return str(self.get_tagged_text())


class ParseTreeNode():
    def flatten(self) -> TaggedText:
        """
        Return the tagged text resulting from left-to-right pass through the tree.
        """
        tagged_text = TaggedText()

        for subnode in self.subnodes:
            if type(subnode) is TaggedToken:
                tagged_text.add_tagged_token(token=subnode.token, tag=subnode.tag)
            else:
                sub_tagged_text = subnode.flatten()
                tagged_text += sub_tagged_text

        return tagged_text


# TODO check if all of these classes are saved in ontology and delete the code
