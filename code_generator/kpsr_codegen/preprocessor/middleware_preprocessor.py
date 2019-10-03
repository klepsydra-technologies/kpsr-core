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

from middleware_definition import RosMiddlewareDefinition
from middleware_definition import DdsMiddlewareDefinition
from middleware_definition import ZmqMiddlewareDefinition

from middleware_type import MiddlewareType


## Middleware Preprocessor Class
# \ingroup codegenerator
class MiddlewarePreprocessor:

    ## The process function
    # @param middleware_data Middleware data
    def process(self, middleware_data):
        middleware_type = middleware_data.get('type')
        mapper_include_file = middleware_data.get('mapper_include_file', '')

        if middleware_type.lower() == 'ros':
            return RosMiddlewareDefinition(MiddlewareType.ROS, mapper_include_file, middleware_data.get('class_name'),
                                           middleware_data.get('already_exists', False), middleware_data.get('include_file', ''),
                                           middleware_data.get('msg_file', ''), middleware_data.get('project_name'),
                                           middleware_data.get('ignore_fields', []))
        if middleware_type.lower() == 'dds':
            return DdsMiddlewareDefinition(MiddlewareType.DDS, mapper_include_file, middleware_data.get('class_name'),
                                           middleware_data.get('already_exists', False), middleware_data.get('include_file', ''),
                                           middleware_data.get('idl_file', ''), middleware_data.get('sequence_fields', []))
        if middleware_type.lower() == 'zmq':
            return ZmqMiddlewareDefinition(MiddlewareType.ZMQ, mapper_include_file,
                                           middleware_data.get('serializer_class_name', ''), middleware_data.get('serializer_include_file', ''))
