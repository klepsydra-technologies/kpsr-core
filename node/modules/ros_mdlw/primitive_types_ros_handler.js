/*
 * Copyright 2023 Klepsydra Technologies AG
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

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

