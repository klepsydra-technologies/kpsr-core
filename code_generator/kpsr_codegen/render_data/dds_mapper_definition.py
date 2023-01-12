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
class DdsMapperFieldDefinition:
    def __init__(self, field_name, field_type, dds_type, is_enum, is_vector, use_raw_pointer, use_smart_pointer,
                 is_array, size, mapper_name):
        self.field_name = field_name
        self.field_type = field_type
        self.dds_type = dds_type
        self.is_enum = is_enum
        self.is_vector = is_vector
        self.use_raw_pointer = use_raw_pointer
        self.use_smart_pointer = use_smart_pointer
        self.is_array = is_array
        self.size = size
        self.mapper_name = mapper_name

    def __repr__(self):
        return "%s(field_name=%r, field_type=%r, dds_type,=%r, is_enum=%r, is_vector=%r, use_raw_pointer=%r, " \
               "use_smart_pointer=%r, is_array=%r, size=%r, mapper_name=%r)" % (
                   self.__class__.__name__, self.field_name, self.dds_type, self.field_type, self.is_enum,
                   self.is_vector, self.use_raw_pointer, self.use_smart_pointer, self.is_array, self.size,
                   self.mapper_name)



class DdsMapperInstanceDefinition:
    def __init__(self, mapper_name, kpsr_class_name, dds_class_name, include_file):
        self.mapper_name = mapper_name
        self.kpsr_class_name = kpsr_class_name
        self.dds_class_name = dds_class_name
        self.include_file = include_file

    def __repr__(self):
        return "%s(mapper_name=%r, kpsr_class_name=%r, dds_class_name=%r, include_file=%r)" % \
               (self.__class__.__name__, self.mapper_name, self.kpsr_class_name, self.dds_class_name,
                self.include_file)


class DdsMapperDefinition:
    def __init__(self, class_name, define_class_name, dds_type, include_list, mapper_instances, fields):
        self.class_name = class_name
        self.define_class_name = define_class_name
        self.dds_type = dds_type
        self.include_list = include_list
        self.mapper_instances = mapper_instances
        self.fields = fields

    def __repr__(self):
        return "%s(class_name=%r, define_class_name=%r, dds_type=%r, include_list=%r, mapper_instances=%r, fields=%r)"\
               % \
               (self.__class__.__name__, self.class_name, self.define_class_name, self.dds_type, self.include_list,
                self.mapper_instances, self.fields)
