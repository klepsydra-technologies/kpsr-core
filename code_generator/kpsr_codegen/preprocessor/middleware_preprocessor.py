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
