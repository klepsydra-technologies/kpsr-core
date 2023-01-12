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
class EnumDefinition:
    def __init__(self, enum_name, values, already_exists, include_file):
        self.enum_name = enum_name
        self.values = values
        self.already_exists = already_exists
        self.include_file = include_file

    def __repr__(self):
        return "%s(enum_name=%r, values=%r, already_exists=%r, include_file=%r)" % (
            self.__class__.__name__, self.enum_name, self.values, self.already_exists, self.include_file)
