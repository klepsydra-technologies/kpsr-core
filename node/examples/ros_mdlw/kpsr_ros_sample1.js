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

/**
 * This example demonstrates simple access to middleware ros through Klepsydra.
 */

'use strict';

// Require rosnodejs itself
const rosnodejs = require('rosnodejs');

var KlepsydraRosConnector = require('../kpsr_ros_connector.js');

var kpsrPrimitiveTypes = require('/opt/klepsydra/node/stages/rosstg/primitive_types_ros_handler.js');
var PoseEventHandler = require('/opt/klepsydra/node/stages/rosstg/pose_event_ros_handler.js');

if (require.main === module) {
  // Invoke Main Listener Function
  var kpsrConf = [
     {
        topicName: '/example_topic/int32',
        handler: new kpsrPrimitiveTypes.IntEventHandler(rosnodejs)
     },
     {
        topicName: '/mavros/local_position/pose',
        handler: new PoseEventHandler(rosnodejs)
     }
  ]

  var klepsydraRosConnector = new KlepsydraRosConnector(rosnodejs, '/listener_node', kpsrConf);
  klepsydraRosConnector.install.then(topicMap => {
     topicMap['/example_topic/int32'].subscriber.on('message', onInt32Event);
     topicMap['/mavros/local_position/pose'].subscriber.on('message', onPoseEvent)
  });
}

function onPoseEvent(data) {
   rosnodejs.log.info('I heard pose: ' + JSON.stringify(data));
}

function onInt32Event(data) {
   rosnodejs.log.info('I heard state: ' + JSON.stringify(data));
}

