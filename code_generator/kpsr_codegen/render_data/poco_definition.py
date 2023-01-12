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
class PocoFieldDefinition:
    def __init__(self, field_name, field_type):
        self.field_name = field_name
        self.field_type = field_type

    def __repr__(self):
        return "%s(field_name=%r, field_type=%r)" % \
               (self.__class__.__name__, self.field_name, self.field_type)


class PocoRelatedClassDefinition:
    def __init__(self, class_name, fields):
        self.class_name = class_name
        self.fields = fields

    def __repr__(self):
        return "%s(class_name=%r, fields=%r)" % \
               (self.__class__.__name__, self.class_name, self.fields)


class PocoEnumDefinition:
    def __init__(self, enum, name, namespaces):
        self.enum_name = enum.enum_name
        self.values = enum.values
        self.already_exists = enum.already_exists
        self.include_file = enum.include_file
        self.name = name
        self.namespaces = namespaces

    def __repr__(self):
        return "%s(enum_name=%r, values=%r, already_exists=%r, include_file=%r, name=%r, namespaces=%r)" % (
            self.__class__.__name__, self.enum_name, self.values, self.already_exists, self.include_file,
            self.name, self.namespaces)


class PocoDefinition:
    def __init__(self, class_name, namespaces, define_class_name, parent_class, system_include_list, custom_include_list, create_builder, fields,
                 enums):
        self.class_name = class_name
        self.namespaces = namespaces
        self.define_class_name = define_class_name
        self.parent_class = parent_class
        self.system_include_list = system_include_list
        self.custom_include_list = custom_include_list
        self.create_builder = create_builder
        self.fields = fields
        self.enums = enums

    def __repr__(self):
        return "%s(class_name=%r, namespaces=%r, define_class_name=%r, parent_class=%r, system_include_list=%r, custom_include_list=%r, " \
               "create_builder=%r, fields=%r, enums=%r)" % \
               (self.__class__.__name__, self.class_name, self.namespaces, self.define_class_name, self.parent_class, self.system_include_list,
                self.custom_include_list, self.create_builder, self.fields, self.enums)
