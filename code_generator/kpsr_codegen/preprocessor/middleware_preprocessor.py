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
