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
