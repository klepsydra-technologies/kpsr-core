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
/**
 * This example demonstrates simple receiving of messages over the ROS system.
 */

// Require rosnodejs itself
const rosnodejs = require('rosnodejs');
// Requires the std_msgs message package
const std_msgs = rosnodejs.require('std_msgs').msg;

const geometry_msgs = rosnodejs.require("geometry_msgs").msg;

const egoi_ros_events_msgs = rosnodejs.require('egoi_ros_events').msg;

function listener(nodeName, kpsrConf) {
  // Register node with ROS master
  rosnodejs.initNode(nodeName)
    .then((rosNode) => {
      kpsrConf.forEach( confData => {
      // Create ROS subscriber on the 'chatter' topic expecting String messages
      let sub = rosNode.subscribe(confData.topic, confData.msgType,
        (data) => { // define callback execution
          confData.callback(data);
        });
        sub.on('error', function() { console.log("on shutdown") });
     });
  });
}

if (require.main === module) {
  // Invoke Main Listener Function
  var kpsrConf = [
     {
        topic: '/lidar_application/lidar_event_topic',
        msgType: egoi_ros_events_msgs.LidarEventMessage,
        callback: onLidarEvent
     },
     {
        topic: '/mavros/local_position/pose',
        msgType: geometry_msgs.PoseStamped,
        callback: onPoseEvent
     },
     {
        topic: '/windsurveyor_application/tm_state_topic',
        msgType: std_msgs.Int32,
        callback: onTMStageEvent
     }
  ]

  listener('/listener_node', kpsrConf);
}

function onPoseEvent(data) {
   rosnodejs.log.info('I heard pose: ' + JSON.stringify(data));
}

function onLidarEvent(data) {
   rosnodejs.log.info('I heard lidar: ' + JSON.stringify(data));
}

function onTMStageEvent(data) {
   rosnodejs.log.info('I heard tm state: ' + JSON.stringify(data));
}

