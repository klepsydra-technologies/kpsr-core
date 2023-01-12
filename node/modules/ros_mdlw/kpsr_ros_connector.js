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

