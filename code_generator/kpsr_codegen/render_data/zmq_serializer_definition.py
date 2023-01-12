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
class SerializerDefinition:
    def __init__(self, serial_define_class_name, class_definition_name, include_list, cereal_include_list, field_names):
        self.serial_define_class_name = serial_define_class_name
        self.class_definition_name = class_definition_name
        self.cereal_include_list = cereal_include_list
        self.include_list = include_list
        self.field_names = field_names

    def __repr__(self):
        return "%s(serial_define_class_name=%r, class_definition_name=%r, include_list=%r, cereal_include_list=%r, " \
               "field_names=%r)" % \
               (self.__class__.__name__, self.serial_define_class_name, self.class_definition_name,
                self.include_list, self.cereal_include_list, self.field_names)
