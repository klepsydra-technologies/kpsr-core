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
