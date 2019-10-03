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
class NodeHandlerDefinition:
    def __init__(self, handler_name, message_name, project_name):
        self.handler_name = handler_name
        self.message_name = message_name
        self.project_name = project_name

    def __repr__(self):
        return "%s(handler_name=%r,message_name=%r, project_name=%r)" % \
               (self.__class__.__name__, self.handler_name, self.message_name, self.project_name)
