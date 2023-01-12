# Copyright 2023 Klepsydra Technologies AG
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


# -*- coding: utf-8 -*-
import re
import sys
import os

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../kidl_data')))

from field_definition import FieldDefinition

## Doc for the FieldPreprocessor class
# \ingroup kpsr-code-generator
#
# Class processes fields in the yaml file.
class FieldPreprocessor:
    def __init__(self, type_modifiers, fundamental_types):
        self.type_modifiers = type_modifiers
        self.fundamental_types = fundamental_types

    ## Process the fields in the kidl file
    #
    # For fundamental data types it does a direct mapping. The matches
    # for pointers (raw or smart) and vectors or arrays are done using
    # regexp. Finally if the data field is none of the above types, it
    # is treated as an enum.

    # @param field The field to be processed.
    # @param enumeration_dict The dictionary of all enums in the file.
    # @param is_zmq_enabled Boolean for zmq as zmq can use smart pointers.
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
