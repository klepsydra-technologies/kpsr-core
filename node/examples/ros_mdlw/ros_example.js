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

