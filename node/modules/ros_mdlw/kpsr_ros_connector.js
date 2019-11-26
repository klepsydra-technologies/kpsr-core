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

