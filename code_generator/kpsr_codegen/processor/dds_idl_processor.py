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

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../kidl_data')))
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../render_data')))

from dds_idl_definition import IdlDefinition
from dds_idl_definition import IdlFieldDefinition
from middleware_type import MiddlewareType
from poco_processor import split_namespace_class
from poco_processor import convert_to_lower_case_underscores


## Doc for the DdsIdlProcessor class
# \ingroup kpsr-code-generator
#
# Processes yaml fields to create DDS Idl files.
class DdsIdlProcessor:

    def __init__(self, configuration):
        self.fundamental_types = configuration.fundamental_types
        self.dds_types = configuration.dds_types
        self.type_modifiers_dds = configuration.type_modifiers_dds

    def process(self, class_definition_name, class_definition_dict):
        class_definition = class_definition_dict.get(class_definition_name)

        idl_field_definitions = [self.process_field(field, class_definition_dict, class_definition.enums)
                                  for field in class_definition.fields]

        include_list = self.process_includes(class_definition, class_definition_dict)

        if MiddlewareType.DDS in class_definition.middlewares:
            dds_middleware_definition = class_definition.middlewares[MiddlewareType.DDS]
            module_name = split_namespace_class(dds_middleware_definition.class_name)[0]
            class_name = split_namespace_class(dds_middleware_definition.class_name)[-1]
            sequence_fields = dds_middleware_definition.sequence_fields
        else:
            sequence_fields = []
            module_name = ''
            class_name = ''

        if class_definition.parent_class is not None:
            parent_class_definition = self.process(class_definition.parent_class, class_definition_dict)
            include_list = parent_class_definition.include_list.union(include_list)
            idl_field_definitions = parent_class_definition.fields + idl_field_definitions

        return IdlDefinition(class_name, module_name, include_list, idl_field_definitions,
                             sequence_fields)

    def process_includes(self, class_definition, class_definition_dict):
        include_list = set()
        for field in class_definition.fields:
            if not field.is_enum and field.field_type not in self.fundamental_types:
                dds_middleware_definition = class_definition_dict.get(field.field_type).middlewares[MiddlewareType.DDS]
                if dds_middleware_definition.idl_file:
                    include_list.add("\"%s\"" % dds_middleware_definition.idl_file)
                else:
                    class_name = split_namespace_class(dds_middleware_definition.class_name)[-1]
                    include_list.add("\"%s.idl\"" % convert_to_lower_case_underscores(class_name))

        return include_list

    def process_field(self, field, class_definition_dict, enums):
        if field.field_type in self.fundamental_types:
            field_dds_type = split_namespace_class(self.dds_types.get(field.field_type))[-1]
        else:
            if field.field_type in enums:
                field_dds_type = self.dds_types.get('enum')
            else:
                dds_middleware_definition = class_definition_dict.get(field.field_type).middlewares[MiddlewareType.DDS]
                field_dds_type = dds_middleware_definition.class_name

        if field.is_vector:
            if field.use_smart_pointer:
                field_dds_type = self.type_modifiers_dds\
                    .get('smart_pointer_vector')\
                    .replace("#0", field.field_name)\
                    .replace("#1", field_dds_type)
            else:
                if field.use_raw_pointer:
                    field_dds_type = self.type_modifiers_dds.get('raw_pointer_vector') \
                        .replace("#0", field.field_name) \
                        .replace("#1", field_dds_type)
                else:
                    field_dds_type = self.type_modifiers_dds.get('vector')\
                        .replace("#0", field.field_name) \
                        .replace("#1", field_dds_type)
        else:
            if field.is_array:
                field_dds_type = self.type_modifiers_dds\
                    .get('fix_size_array') \
                    .replace("#0", field.field_name) \
                    .replace("#1", field_dds_type)\
                    .replace("#2", field.size)
            else:
                field_dds_type = "%s %s" % (field_dds_type, field.field_name)
        return IdlFieldDefinition(field.field_name, field_dds_type)
