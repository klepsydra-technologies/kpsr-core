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
class ClassDefinition:
    def __init__(self, class_name, already_exists, create_builder, include_file, parent_class, enums, middlewares, fields):
        self.class_name = class_name
        self.already_exists = already_exists
        self.create_builder = create_builder
        self.include_file = include_file
        self.parent_class = parent_class
        self.enums = enums
        self.middlewares = middlewares
        self.fields = fields

    def __repr__(self):
        return "%s(class_name=%r, already_exists=%r, create_builder=%r, include_file=%r, parent_class=%r, enums=%r, " \
               "middlewares=%r, fields=%r)" % (
            self.__class__.__name__, self.class_name, self.already_exists, self.create_builder, self.include_file,
            self.parent_class, self.enums, self.middlewares, self.fields)
