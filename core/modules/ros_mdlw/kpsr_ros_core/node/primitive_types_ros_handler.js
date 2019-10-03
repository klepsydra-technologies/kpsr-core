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

'use strict';

function BooleanEventHandler(rosnodejs) {
   var std_msgs = rosnodejs.require('std_msgs').msg;
   this.msgType = std_msgs.Bool;
   this.from = function (data) { return data; };
   this.to = function (data) { return data; };
}

function IntEventHandler(rosnodejs) {
   var std_msgs = rosnodejs.require('std_msgs').msg;
   this.msgType = std_msgs.Int32;
   this.from = function (data) { return data; };
   this.to = function (data) { return data; };
}

function LongEventHandler(rosnodejs) {
   var std_msgs = rosnodejs.require('std_msgs').msg;
   this.msgType = std_msgs.Int64;
   this.from = function (data) { return data; };
   this.to = function (data) { return data; };
}

function FloatEventHandler(rosnodejs) {
   var std_msgs = rosnodejs.require('std_msgs').msg;
   this.msgType = std_msgs.Float32;
   this.from = function (data) { return data; };
   this.to = function (data) { return data; };
}

function DoubleEventHandler(rosnodejs) {
   var std_msgs = rosnodejs.require('std_msgs').msg;
   this.msgType = std_msgs.Float64;
   this.from = function (data) { return data; };
   this.to = function (data) { return data; };
}

function StringEventHandler(rosnodejs) {
   var std_msgs = rosnodejs.require('std_msgs').msg;
   this.msgType = std_msgs.String;
   this.from = function (data) { return data; };
   this.to = function (data) { return data; };
}

module.exports = {
   BooleanEventHandler: BooleanEventHandler,
   IntEventHandler: IntEventHandler,
   LongEventHandler: LongEventHandler,
   FloatEventHandler: FloatEventHandler,
   DoubleEventHandler: DoubleEventHandler,
   StringEventHandler: StringEventHandler
}

