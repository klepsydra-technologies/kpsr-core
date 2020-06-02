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
import sys
import os

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../kserializer_data')))
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../render_data')))

from zmq_serializer_definition import SerializerDefinition
from middleware_type import MiddlewareType
from poco_processor import split_namespace_class
from poco_processor import convert_to_lower_case_underscores


## Doc for the ZmqSerializerProcessor class
# \ingroup kpsr-code-generator
#
# Processes yaml fields to create a serialized files for ZMQ using Cereal.
class ZmqSerializerProcessor:

    def __init__(self, configuration):
        self.fundamental_types = configuration.fundamental_types

    def process(self, class_definition_name, class_definition_dict, include_path):
        class_definition = class_definition_dict.get(class_definition_name)

        serializer_field_definitions = [field.field_name for field in class_definition.fields]

        include_list = self.process_includes(class_definition, class_definition_dict, include_path)
        cereal_include_list = self.process_cereal_includes(class_definition)

        if class_definition.parent_class is not None:
            parent_class_definition = self.process(class_definition.parent_class, class_definition_dict, include_path)
            include_list = parent_class_definition.include_list.union(include_list)
            cereal_include_list = parent_class_definition.cereal_include_list.union(cereal_include_list)
            serializer_field_definitions = parent_class_definition.field_names + serializer_field_definitions

        serializer_class_name = "%sSerializer" % split_namespace_class(class_definition_name)[-1]

        return SerializerDefinition(convert_to_lower_case_underscores(serializer_class_name).upper(),
                                    class_definition_name, include_list, cereal_include_list,
                                    serializer_field_definitions)

    def process_includes(self, class_definition, class_definition_dict, include_path):
        include_list = set()
        if class_definition.already_exists:
            include_list.add(class_definition.include_file)
        else:
            class_name = split_namespace_class(class_definition.class_name)[-1]
            if include_path:
                include = "<%s/%s.h>" % (include_path, convert_to_lower_case_underscores(class_name))
                include_list.add(include.replace("//", "/"))
            else:
                include_list.add("<%s.h>" % convert_to_lower_case_underscores(class_name))

        for field in class_definition.fields:
            if not field.is_enum and field.field_type not in self.fundamental_types:
                zmq_middleware_definition = class_definition_dict.get(field.field_type).middlewares[MiddlewareType.ZMQ]
                if zmq_middleware_definition.serializer_include_file:
                    include_list.add(zmq_middleware_definition.serializer_include_file)
                else:
                    class_name = split_namespace_class(field.field_type)[-1]
                    if include_path:
                        include = "<%s/cereal/%s_serializer.h>" % (include_path,
                                                                   convert_to_lower_case_underscores(class_name))
                        include_list.add(include.replace("//", "/"))
                    else:
                        include = "<cereal/%s.h>" % convert_to_lower_case_underscores(class_name)
                        include_list.add(include.replace("//", "/"))

        return include_list

    def process_cereal_includes(self, class_definition):
        include_list = set()
        for field in class_definition.fields:
            if field.is_vector:
                include_list.add("<cereal/types/vector.hpp>")
            if field.use_smart_pointer:
                include_list.add("<cereal/types/memory.hpp>")
            if field.is_array:
                include_list.add("<cereal/types/array.hpp>")
            if field.field_type == 'string':
                include_list.add("<cereal/types/string.hpp>")

        return include_list
