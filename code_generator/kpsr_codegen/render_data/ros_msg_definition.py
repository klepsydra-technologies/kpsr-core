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
class MsgFieldDefinition:
    def __init__(self, field_name, field_type):
        self.field_name = field_name
        self.field_type = field_type

    def __repr__(self):
        return "%s(field_name=%r, field_type=%r)" % \
               (self.__class__.__name__, self.field_name, self.field_type)


class MsgDefinition:
    def __init__(self, class_name, parent_class, fields):
        self.class_name = class_name
        self.parent_class = parent_class
        self.fields = fields

    def __repr__(self):
        return "%s(class_name=%r, parent_class=%r, fields=%r)" % \
               (self.__class__.__name__, self.class_name, self.parent_class, self.fields)
