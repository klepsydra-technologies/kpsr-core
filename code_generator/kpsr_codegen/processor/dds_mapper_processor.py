#****************************************************************************
#
#                           Klepsydra Core Modules
#              Copyright (C) 2019-2020  Klepsydra Technologies GmbH
#                            All Rights Reserved.
#
#  This file is subject to the terms and conditions defined in
#  file 'LICENSE.md', which is part of this source code package.
#
#  NOTICE:  All information contained herein is, and remains the property of Klepsydra
#  Technologies GmbH and its suppliers, if any. The intellectual and technical concepts
#  contained herein are proprietary to Klepsydra Technologies GmbH and its suppliers and
#  may be covered by Swiss and Foreign Patents, patents in process, and are protected by
#  trade secret or copyright law. Dissemination of this information or reproduction of
#  this material is strictly forbidden unless prior written permission is obtained from
#  Klepsydra Technologies GmbH.
#
#****************************************************************************

# -*- coding: utf-8 -*-

import re
import sys
import os

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../kidl_data')))
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../render_data')))
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../processor')))

from middleware_type import MiddlewareType
from dds_mapper_definition import DdsMapperInstanceDefinition
from dds_mapper_definition import DdsMapperFieldDefinition
from dds_mapper_definition import DdsMapperDefinition
from poco_processor import convert_to_lower_case_underscores


def process_custom_includes(class_definition, mapper_instances, include_path):
    custom_includes = set()

    for mapper_instance in mapper_instances:
        if mapper_instance.include_file:
            custom_includes.add(mapper_instance.include_file)

    if class_definition.already_exists:
        custom_includes.add(class_definition.include_file)
    else:
        class_name = split_namespace_class(class_definition.class_name)[-1]
        if include_path:
            custom_includes.add("<%s%s.h>" % (include_path, convert(class_name)))
        else:
            custom_includes.add("<%s.h>" % convert(class_name))

    middleware_definition = class_definition.middlewares[MiddlewareType.DDS]
    if middleware_definition.already_exists:
        custom_includes.add(middleware_definition.include_file)
    else:
        class_name = split_namespace_class(middleware_definition.class_name)[-1]
        custom_includes.add("\"%s.hpp\"" % convert_to_lower_case_underscores(class_name))

    return custom_includes


def split_namespace_class(class_name):
    return class_name.split("::")


def camelCase(string):
    output = ''.join(x for x in string if x.isalpha())
    return output[0].lower() + output[1:]


def process_mapper_instances(class_definition, class_definition_dict, include_path):
    mapper_instances = dict()

    for field in class_definition.fields:
        if field.is_related_class:
            dds_middleware_definition = class_definition_dict.get(field.field_type).middlewares[MiddlewareType.DDS]
            field_dds_type = dds_middleware_definition.class_name
            dds_type = field_dds_type
            mapper_name = '_%s_mapper' % field.field_type.replace('::', '_').lower()

            if dds_middleware_definition.mapper_include_file:
                include_file = dds_middleware_definition.mapper_include_file
            else:
                class_name = split_namespace_class(field.field_type)[-1]
                include_file = "<%s/dds/%s%s>" % (include_path, convert_to_lower_case_underscores(class_name),
                                                     "_dds_mapper.h")
                include_file = include_file.replace("//", "/")

            dds_mapper_definition = DdsMapperInstanceDefinition(mapper_name, field.field_type, dds_type, include_file)

            mapper_instances[mapper_name] = dds_mapper_definition

    return list(mapper_instances.values())


def convert(name):
    s1 = re.sub('(.)([A-Z][a-z]+)', r'\1_\2', name)
    return re.sub('([a-z0-9])([A-Z])', r'\1_\2', s1).lower()


## Doc for the DDSMapperProcessor class
# \ingroup kpsr-code-generator
#
# Processes yaml fields to map them to dds compatible types.
class DdsMapperProcessor:

    def __init__(self, configuration):
        self.dds_types = configuration.dds_types
        self.fundamental_types = configuration.fundamental_types

    def process(self, class_definition_name, class_definition_dict, include_path):
        class_definition = class_definition_dict.get(class_definition_name)

        define_class_name = convert('%sMapper' % split_namespace_class(class_definition.class_name)[-1]).upper()
        mapper_instances = process_mapper_instances(class_definition, class_definition_dict, include_path)
        custom_includes = process_custom_includes(class_definition, mapper_instances, include_path)
        field_definitions = [self.process_field(field, class_definition_dict) for field in class_definition.fields]
        if class_definition.parent_class is not None:
            parent_class_definition = class_definition_dict.get(class_definition.parent_class)
            parent_field_definitions = [self.process_field(field, class_definition_dict)
                                        for field in parent_class_definition.fields]
            field_definitions = field_definitions + parent_field_definitions

        dds_middleware_definition = class_definition.middlewares[MiddlewareType.DDS]
        field_dds_type = dds_middleware_definition.class_name
        dds_type = field_dds_type

        return DdsMapperDefinition(class_definition.class_name, define_class_name , dds_type, custom_includes,
                                   mapper_instances, field_definitions)

    def process_field(self, field, class_definition_dict):
        if field.is_related_class:
            mapper_name = '_%s_mapper' % field.field_type.replace('::', '_').lower()
        else:
            mapper_name = None

        if field.is_related_class:
            dds_middleware_definition = class_definition_dict.get(field.field_type).middlewares[MiddlewareType.DDS]
            field_dds_type = dds_middleware_definition.class_name
        elif field.is_enum:
            field_dds_type = self.dds_types.get('int16')
        else:
            field_dds_type = self.dds_types.get(field.field_type)

        if field.field_type in self.fundamental_types:
            field_cpp_type = self.fundamental_types.get(field.field_type)
        else:
            field_cpp_type = field.field_type

        return DdsMapperFieldDefinition(field.field_name, field_cpp_type, field_dds_type, field.is_enum,
                                        field.is_vector, field.use_raw_pointer, field.use_smart_pointer,
                                        field.is_array, field.size, mapper_name)

