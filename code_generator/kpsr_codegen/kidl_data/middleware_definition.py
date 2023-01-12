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
