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
class SerializerDefinition:
    def __init__(self, serial_define_class_name, class_definition_name, include_list, cereal_include_list, field_names):
        self.serial_define_class_name = serial_define_class_name
        self.class_definition_name = class_definition_name
        self.cereal_include_list = cereal_include_list
        self.include_list = include_list
        self.field_names = field_names

    def __repr__(self):
        return "%s(serial_define_class_name=%r, class_definition_name=%r, include_list=%r, cereal_include_list=%r, " \
               "field_names=%r)" % \
               (self.__class__.__name__, self.serial_define_class_name, self.class_definition_name,
                self.include_list, self.cereal_include_list, self.field_names)
