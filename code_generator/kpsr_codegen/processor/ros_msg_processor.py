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
import sys
import os

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../kidl_data')))
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../render_data')))

from ros_msg_definition import MsgDefinition
from ros_msg_definition import MsgFieldDefinition
from middleware_type import MiddlewareType
from poco_processor import split_namespace_class

## Doc for the RosMsgProcessor class
# \ingroup kpsr-code-generator
#
# Processes the yaml data to create ROS msg files.
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
