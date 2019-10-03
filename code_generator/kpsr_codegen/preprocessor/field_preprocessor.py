# Klepsydra Core Modules
# Copyright (C) 2019-2020  Klepsydra Technologies GmbH
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

# -*- coding: utf-8 -*-
import re
import sys
import os

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../kidl_data')))

from field_definition import FieldDefinition


class FieldPreprocessor:
    def __init__(self, type_modifiers, fundamental_types):
        self.type_modifiers = type_modifiers
        self.fundamental_types = fundamental_types

    def process(self, field, enumeration_dict, is_zmq_enabled):
        field_type = field['type']
        is_vector = False
        use_raw_pointer = False
        use_smart_pointer = False
        is_array = False
        size = 0
        is_related_class = False
        is_enum = False

        vector_regexp = re.compile(self.type_modifiers['vector'])
        is_match = vector_regexp.match(field_type)
        if is_match:
            field_type = is_match.group(1)
            is_vector = True
        else:
            smart_pointer_vector_regexp = re.compile(self.type_modifiers['smart_pointer_vector'])
            is_match = smart_pointer_vector_regexp.match(field_type)
            if is_match:
                field_type = is_match.group(1)
                is_vector = True
                use_smart_pointer = True

            else:
                raw_pointer_vector_regexp = re.compile(self.type_modifiers['raw_pointer_vector'])
                is_match = raw_pointer_vector_regexp.match(field_type)
                if is_match:
                    field_type = is_match.group(1)
                    is_vector = True
                    if not is_zmq_enabled:
                        use_raw_pointer = True
                    else:
                        use_smart_pointer = True

                else:
                    fix_size_array_regexp = re.compile(self.type_modifiers['fix_size_array'])
                    is_match = fix_size_array_regexp.match(field_type)
                    if is_match:
                        field_type = is_match.group(1)
                        is_array = True
                        size = is_match.group(2)

        if field_type not in self.fundamental_types:
            if field_type in enumeration_dict:
                is_enum = True
            else:
                is_related_class = True

        return FieldDefinition(field['name'], field_type, is_vector, use_raw_pointer, use_smart_pointer, is_array,
                               size, is_related_class, is_enum)
