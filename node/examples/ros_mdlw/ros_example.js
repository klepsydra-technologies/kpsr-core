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

