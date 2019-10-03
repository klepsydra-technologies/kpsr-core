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
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../render_data')))

from ros_msg_definition import MsgDefinition
from ros_msg_definition import MsgFieldDefinition
from middleware_type import MiddlewareType
from poco_processor import split_namespace_class


class RosMsgProcessor:

    def __init__(self, configuration):
        self.fundamental_types = configuration.fundamental_types
        self.ros_types = configuration.ros_types
        self.type_modifiers_ros = configuration.type_modifiers_ros

    def process(self, class_definition_name, class_definition_dict):
        class_definition = class_definition_dict.get(class_definition_name)

        ros_middleware_definition = class_definition.middlewares.get(MiddlewareType.ROS)
        if ros_middleware_definition is None:
            ros_fields = class_definition.fields
        else:
            ros_fields = [field for field in class_definition.fields if field.field_name
                          not in ros_middleware_definition.ignore_fields]
        poco_field_definitions = [self.process_field(field, class_definition_dict, class_definition.enums)
                                  for field in ros_fields]
        if class_definition.parent_class is not None:
            parent_class_definition = self.process(class_definition.parent_class, class_definition_dict)
        else:
            parent_class_definition = None

        return MsgDefinition(class_definition.class_name, parent_class_definition, poco_field_definitions)

    def process_field(self, field, class_definition_dict, enums):
        if field.field_type in self.fundamental_types:
            project_name = None
            field_ros_type = split_namespace_class(self.ros_types.get(field.field_type))[-1]
        else:
            if field.field_type in enums:
                project_name = None
                field_ros_type = self.ros_types.get('enum')
            else:
                ros_middleware_definition = class_definition_dict.get(field.field_type).middlewares[MiddlewareType.ROS]
                project_name = ros_middleware_definition.project_name
                field_ros_type = ros_middleware_definition.class_name

        if field.is_vector:
            if field.use_smart_pointer:
                field_ros_type = self.type_modifiers_ros.get('smart_pointer_vector').replace("#1", field_ros_type)
            else:
                if field.use_raw_pointer:
                    field_ros_type = self.type_modifiers_ros.get('raw_pointer_vector').replace("#1", field_ros_type)
                else:
                    field_ros_type = self.type_modifiers_ros.get('vector').replace("#1", field_ros_type)
        else:
            if field.is_array:
                field_ros_type = self.type_modifiers_ros.get('fix_size_array').replace("#1", field_ros_type).replace("#2", field.size)

        if project_name is not None:
            field_ros_type = "%s/%s" % (project_name, field_ros_type)

        return MsgFieldDefinition(field.field_name, field_ros_type)
