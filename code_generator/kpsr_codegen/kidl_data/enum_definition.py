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
class EnumDefinition:
    def __init__(self, enum_name, values, already_exists, include_file):
        self.enum_name = enum_name
        self.values = values
        self.already_exists = already_exists
        self.include_file = include_file

    def __repr__(self):
        return "%s(enum_name=%r, values=%r, already_exists=%r, include_file=%r)" % (
            self.__class__.__name__, self.enum_name, self.values, self.already_exists, self.include_file)
