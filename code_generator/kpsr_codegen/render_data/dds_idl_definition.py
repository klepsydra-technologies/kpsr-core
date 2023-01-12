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
class IdlFieldDefinition:
    def __init__(self, field_name, field_type):
        self.field_name = field_name
        self.field_type = field_type

    def __repr__(self):
        return "%s(field_name=%r, field_type=%r)" % \
               (self.__class__.__name__, self.field_name, self.field_type)


class IdlDefinition:
    def __init__(self, class_name, module_name, include_list, fields, sequence_fields):
        self.class_name = class_name
        self.module_name = module_name
        self.include_list = include_list
        self.fields = fields
        self.sequence_fields = sequence_fields

    def __repr__(self):
        return "%s(class_name=%r, module_name=%r, include_list=%r, fields=%r, sequence_fields=%r)" % \
               (self.__class__.__name__, self.class_name, self.module_name, self.include_list, self.fields,
                self.sequence_fields)
