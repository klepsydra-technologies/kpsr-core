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


class MiddlewareDefinition:
    def __init__(self, middleware_type, mapper_include_file):
        self.middleware_type = middleware_type
        self.mapper_include_file = mapper_include_file

    def __repr__(self):
        return "%s(middleware_type=%r, mapper_include_file=%r)" % (self.__class__.__name__, self.middleware_type, self.mapper_include_file)


class RosMiddlewareDefinition(MiddlewareDefinition):
    def __init__(self, middleware_type, mapper_include_file, class_name, already_exists, include_file, msg_file,
                 project_name, ignore_fields):
        MiddlewareDefinition.__init__(self, middleware_type, mapper_include_file)
        self.class_name = class_name
        self.already_exists = already_exists
        self.include_file = include_file
        self.msg_file = msg_file
        self.project_name = project_name
        self.ignore_fields = ignore_fields

    def __repr__(self):
        return "%s(middleware_type=%r, mapper_include_file=%r, class_name=%r, already_exists=%r, include_file=%r, " \
               "msg_file=%r, project_name=%r, ignore_fields=%r)" \
               % (self.__class__.__name__, self.middleware_type, self.mapper_include_file, self.class_name,
                  self.already_exists, self.include_file, self.msg_file, self.project_name, self.ignore_fields)


class DdsMiddlewareDefinition(MiddlewareDefinition):
    def __init__(self, middleware_type, mapper_include_file, class_name, already_exists, include_file, idl_file,
                 sequence_fields):
        MiddlewareDefinition.__init__(self, middleware_type, mapper_include_file)
        self.class_name = class_name
        self.already_exists = already_exists
        self.include_file = include_file
        self.idl_file = idl_file
        self.sequence_fields = sequence_fields

    def __repr__(self):
        return "%s(middleware_type=%r, mapper_include_file=%r, class_name=%r, already_exists=%r, include_file=%r, " \
               "idl_file=%r, sequence_fields=%r)" \
               % (self.__class__.__name__, self.middleware_type, self.mapper_include_file, self.class_name,
                  self.already_exists, self.include_file, self.idl_file, self.sequence_fields)


class ZmqMiddlewareDefinition(MiddlewareDefinition):
    def __init__(self, middleware_type, mapper_include_file, serializer_class_name, serializer_include_file):
        MiddlewareDefinition.__init__(self, middleware_type, mapper_include_file)
        self.serializer_class_name = serializer_class_name
        self.serializer_include_file = serializer_include_file

    def __repr__(self):
        return "%s(middleware_type=%r, mapper_include_file=%r, serializer_class_name=%r, serializer_include_file=%r)" \
               % (self.__class__.__name__, self.middleware_type, self.mapper_include_file, self.serializer_class_name,
                  self.serializer_include_file)
