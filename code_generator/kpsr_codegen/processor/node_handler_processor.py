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
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../processor')))

from middleware_type import MiddlewareType
from node_handler_definition import NodeHandlerDefinition
from poco_processor import split_namespace_class


class NodeHandlerProcessor:

    def __init__(self, configuration):
        self.fundamental_types = configuration.fundamental_types

    def process(self, class_definition_name, class_definition_dict):
        class_definition = class_definition_dict.get(class_definition_name)
        ros_middleware_definition = class_definition.middlewares[MiddlewareType.ROS]

        handler_name = split_namespace_class(class_definition.class_name)[-1]+"Handler"
        message_name = ros_middleware_definition.class_name
        project_name = ros_middleware_definition.project_name

        return NodeHandlerDefinition(handler_name, message_name, project_name)
