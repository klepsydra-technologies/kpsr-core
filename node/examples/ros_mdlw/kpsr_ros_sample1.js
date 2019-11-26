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

