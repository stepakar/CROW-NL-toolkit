#!/usr/bin/env python
"""
Copyright (c) 2019 CIIRC, CTU in Prague
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.

@author: Zdenek Kasner
"""
from speech_vr.encoder.MorphCategory import POS





# with db.onto as onto:
class Tag():
    """
    Represents positional tags e.g. as defined here - http://ufal.mff.cuni.cz/pdt2.0/doc/manuals/en/m-layer/html/ch02s02s01.html
    TODO: extend for other tags than POS
    """
    def __str__(self):
        return f"{self.pos}"

    def equals(self, other, include_pos_subcategories : bool = True):
        # TODO make it work with all categories

        if include_pos_subcategories:
            return self.pos.id.startswith(other.pos.id)

        return self.pos.id == other.pos.id


# TODO check if this class is saved in ontology and delete the code
