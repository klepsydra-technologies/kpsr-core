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
