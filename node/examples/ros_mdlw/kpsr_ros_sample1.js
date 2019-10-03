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

