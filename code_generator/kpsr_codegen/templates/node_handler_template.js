/****************************************************************************
*
*                           Klepsydra Core Modules
*              Copyright (C) 2019-2020  Klepsydra Technologies GmbH
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*
****************************************************************************/

// This code has been automatically generated, manual modification might be inadvertently overridden.

'use strict';

function {{definition.handler_name}}(rosnodejs) {
   var messages = rosnodejs.require('{{definition.project_name}}').msg;
   this.msgType = messages.{{definition.message_name}};
   this.from = function (data) { return data; };
   this.to = function (data) { return data; };
}

module.exports = {{definition.handler_name}};
