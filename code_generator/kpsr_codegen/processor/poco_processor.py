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
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../render_data')))

from poco_definition import PocoDefinition
from poco_definition import PocoFieldDefinition
from poco_definition import PocoEnumDefinition


def process_system_includes(class_definition):
    system_includes = set()
    for field in class_definition.fields:
        if field.is_vector:
            system_includes.add("<vector>")
        if field.use_smart_pointer:
            system_includes.add("<memory>")
        if field.field_type == "string":
            system_includes.add("<string>")

    if class_definition.create_builder:
        system_includes.add("<memory>")

    return system_includes


def convert_to_lower_case_underscores(name):
    s1 = re.sub('(.)([A-Z][a-z]+)', r'\1_\2', name)
    return re.sub('([a-z0-9])([A-Z])', r'\1_\2', s1).lower()


def process_custom_includes(class_definition, class_definition_dict, include_path):
    custom_includes = set()

    if class_definition.parent_class is not None:
        parent_class_definition = class_definition_dict.get(class_definition.parent_class)
        if parent_class_definition.already_exists:
            custom_includes.add(parent_class_definition.include_file)
        else:
            parent_class_name = split_namespace_class(class_definition.parent_class)[-1]
            if include_path:
                custom_include = "<%s/%s.h>" % (include_path, convert_to_lower_case_underscores(parent_class_name))
                custom_includes.add(custom_include.replace("//", "/"))
            else:
                custom_includes.add("<%s.h>" % convert_to_lower_case_underscores(parent_class_name))

    for field in class_definition.fields:
        if field.is_enum:
            enum_definition = class_definition.enums.get(field.field_type)
            if enum_definition.already_exists:
                include_file = "\"%s\"" % enum_definition.include_file
                custom_includes.add(include_file.replace("\"<", "<").replace(">\"", ">").replace("\"\"", "\""))

        if field.is_related_class:
            related_class_definition = class_definition_dict.get(field.field_type)
            if related_class_definition.already_exists:
                include_file = "\"%s\"" % related_class_definition.include_file
                custom_includes.add(include_file.replace("\"<", "<").replace(">\"", ">").replace("\"\"", "\""))
            else:
                related_class_name = split_namespace_class(field.field_type)[-1]
                if include_path:
                    custom_include = "<%s/%s.h>" % (include_path, convert_to_lower_case_underscores(related_class_name))
                    custom_includes.add(custom_include.replace("//", "/"))
                else:
                    custom_includes.add("<%s.h>" % convert_to_lower_case_underscores(related_class_name))

    return custom_includes


def split_namespace_class(class_name):
    return class_name.split("::")

## Doc for the PocoProcessor class
# \ingroup kpsr-code-generator
#
# Processes yaml fields to create a Plain Old C Object (poco).
class PocoProcessor:

    def __init__(self, configuration):
        self.fundamental_types = configuration.fundamental_types
        self.type_modifiers_cpp = configuration.type_modifiers_cpp

    def process(self, class_definition_name, class_definition_dict, include_path):
        class_definition = class_definition_dict.get(class_definition_name)

        system_includes = process_system_includes(class_definition)
        custom_includes = process_custom_includes(class_definition, class_definition_dict, include_path)

        poco_field_definitions = [self.process_field(field) for field in class_definition.fields]

        if class_definition.parent_class is not None:
            parent_poco_definition = self.process(class_definition.parent_class, class_definition_dict, include_path)
        else:
            parent_poco_definition = None

        poco_enum_dict = {enum_name: self.process_enum_data(enumeration) for enum_name, enumeration
                          in class_definition.enums.items()}

        namespaces = split_namespace_class(class_definition.class_name)
        namespace_class = namespaces[-1]
        del(namespaces[-1])
        define_class_name = convert_to_lower_case_underscores(split_namespace_class(class_definition.class_name)[-1]).upper()
        return PocoDefinition(namespace_class, namespaces, define_class_name, parent_poco_definition, system_includes,
                              custom_includes, class_definition.create_builder, poco_field_definitions, poco_enum_dict)

    def process_field(self, field):
        if field.field_type in self.fundamental_types:
            field_cpp_type = self.fundamental_types.get(field.field_type)
        else:
            field_cpp_type = field.field_type

        if field.is_vector:
            if field.use_smart_pointer:
                field_cpp_type = self.type_modifiers_cpp.get('smart_pointer_vector').replace("#1", field_cpp_type)
            else:
                if field.use_raw_pointer:
                    field_cpp_type = self.type_modifiers_cpp.get('raw_pointer_vector').replace("#1", field_cpp_type)
                else:
                    field_cpp_type = self.type_modifiers_cpp.get('vector').replace("#1", field_cpp_type)
        else:
            if field.is_array:
                field_cpp_type = self.type_modifiers_cpp.get('fix_size_array').replace("#1", field_cpp_type).replace("#2", field.size)

        return PocoFieldDefinition(field.field_name, field_cpp_type)

    def process_enum_data(self, enum):
        namespaces = split_namespace_class(enum.enum_name)
        name = namespaces[-1]
        del(namespaces[-1])
        return PocoEnumDefinition(enum, name, namespaces)
