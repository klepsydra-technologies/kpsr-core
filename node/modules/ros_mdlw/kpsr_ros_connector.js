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

var events = require('events');

function KlepsydraRosConnector(rosnodejs, nodeName, kpsrConf) {

  var topicProviderMap = {};

  // Register node with ROS master
  this.install = rosnodejs.initNode(nodeName)
    .then((rosNode) => {
      kpsrConf.forEach( confData => {
      // Create subscriber
      let eventEmitter = new events.EventEmitter();
      let sub = rosNode.subscribe(confData.topicName, confData.handler.msgType,
        (data) => { // define callback execution
            eventEmitter.emit('message', confData.handler.from(data));
        });

      // Create publisher
      let rosPublisher = rosNode.advertise(confData.topicName, confData.handler.msgType);
      let publishFunction = function publish(msg) {
         rosPublisher.publish(confData.handler.to(msg));
      }

      // Add subscriber & publisher to topic map.
      let pubSubPair = { subscriber: eventEmitter, publish: publishFunction };
      topicProviderMap[confData.topicName] = pubSubPair;
     });
     return topicProviderMap;
  });
}

module.exports = KlepsydraRosConnector;

