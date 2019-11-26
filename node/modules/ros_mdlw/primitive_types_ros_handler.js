/****************************************************************************
*
*                           Klepsydra Core Modules
*              Copyright (C) 2019-2020  Klepsydra Technologies GmbH
*                            All Rights Reserved.
*
*  This file is subject to the terms and conditions defined in
*  file 'LICENSE.md', which is part of this source code package.
*
*  NOTICE:  All information contained herein is, and remains the property of Klepsydra
*  Technologies GmbH and its suppliers, if any. The intellectual and technical concepts
*  contained herein are proprietary to Klepsydra Technologies GmbH and its suppliers and
*  may be covered by Swiss and Foreign Patents, patents in process, and are protected by
*  trade secret or copyright law. Dissemination of this information or reproduction of
*  this material is strictly forbidden unless prior written permission is obtained from
*  Klepsydra Technologies GmbH.
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

