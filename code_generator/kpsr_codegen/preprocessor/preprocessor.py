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
