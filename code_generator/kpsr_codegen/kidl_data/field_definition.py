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
class FieldDefinition:
    def __init__(self, field_name, field_type, is_vector, use_raw_pointer, use_smart_pointer, is_array, size,
                 is_related_class, is_enum):
        self.field_name = field_name
        self.field_type = field_type
        self.is_vector = is_vector
        self.use_raw_pointer = use_raw_pointer
        self.use_smart_pointer = use_smart_pointer
        self.is_array = is_array
        self.size = size
        self.is_related_class = is_related_class
        self.is_enum = is_enum

    def __repr__(self):
        return "%s(field_name=%r, field_type=%r, is_vector=%r, use_raw_pointer=%r, use_smart_pointer=%r, is_array=%r, size=%r, " \
               "is_related_class=%r, is_enum=%r)" % (
            self.__class__.__name__, self.field_name, self.field_type, self.is_vector, self.use_raw_pointer, self.use_smart_pointer,
            self.is_array, self.size, self.is_related_class, self.is_enum)
