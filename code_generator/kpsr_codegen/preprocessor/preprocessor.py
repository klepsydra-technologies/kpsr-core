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

from class_definition import ClassDefinition
from enum_definition import EnumDefinition
from middleware_type import MiddlewareType

from field_preprocessor import FieldPreprocessor

from middleware_preprocessor import MiddlewarePreprocessor

## Doc for the Preprocessor class
# \ingroup kpsr-code-generator
#
# 
class Preprocessor:

    def __init__(self, configuration):
        self.fieldProcessor = FieldPreprocessor(configuration.type_modifiers, configuration.fundamental_types)
        self.middlewarePreprocessor = MiddlewarePreprocessor()

    ## Process the Yaml Object into ClassDefinition object
    #
    # @param class_definition_data python object returned by yaml.load on reading the kidl file.
    # @param disable_zmq Boolean on whether to enable for ZMQ or not.
    #
    # @return data from file as a ClassDefinition object
    def process(self, class_definition_data, disable_zmq):
        if class_definition_data.get('class_name') is None:
            related_classes = [self.process_related_class(related_class) for related_class in class_definition_data.get('related_classes', [])]
            related_classes_dict = {related_class.class_name: related_class for related_class in related_classes}
            return related_classes_dict

        enumerations = [self.process_enum(enum) for enum in class_definition_data.get('enums', [],)]
        enumeration_dict = {enumeration.enum_name: enumeration for enumeration in enumerations}

        middleware_definitions = [self.middlewarePreprocessor.process(middleware) for middleware in class_definition_data.get('middlewares', [])]
        middleware_definition_dict = {middleware_definition.middleware_type: middleware_definition
                                      for middleware_definition in middleware_definitions}


        is_zmq_enable = (not disable_zmq) and MiddlewareType.ZMQ in middleware_definition_dict
        processed_fields = [self.fieldProcessor.process(field, enumeration_dict, is_zmq_enable)
                            for field in class_definition_data.get('fields', [])]

        return ClassDefinition(class_definition_data.get('class_name'),
                               class_definition_data.get('already_exists', False),
                               class_definition_data.get('create_builder', False),
                               class_definition_data.get('include_file', ''),
                               class_definition_data.get('parent_class', None),
                               enumeration_dict, middleware_definition_dict, processed_fields)

    ## Process enum data types
    #
    # @param enum Enum field in the yaml file.
    def process_enum(self, enum):
        return EnumDefinition(enum.get('enum_name'), enum.get('values'),
                              enum.get('already_exists', False), enum.get('include_file', ''))

    ## Process existing related classes
    #
    # @param related_class Related_class field in the yaml file
    def process_related_class(self, related_class):
        class_name = related_class.get('class_name')
        already_exists = True
        create_builder = False
        include_file = related_class.get('include_file')
        parent_class = None
        middleware_definitions = [self.middlewarePreprocessor.process(middleware) for middleware in related_class.get('middlewares', [])]
        middleware_definition_dict = {middleware_definition.middleware_type: middleware_definition
                                      for middleware_definition in middleware_definitions}
        
        return ClassDefinition(class_name,
                               already_exists,
                               create_builder,
                               include_file,
                               parent_class,
                               {}, middleware_definition_dict, [])
