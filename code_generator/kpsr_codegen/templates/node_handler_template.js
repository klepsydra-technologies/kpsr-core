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

// This code has been automatically generated, manual modification might be inadvertently overridden.

'use strict';

function {{definition.handler_name}}(rosnodejs) {
   var messages = rosnodejs.require('{{definition.project_name}}').msg;
   this.msgType = messages.{{definition.message_name}};
   this.from = function (data) { return data; };
   this.to = function (data) { return data; };
}

module.exports = {{definition.handler_name}};
