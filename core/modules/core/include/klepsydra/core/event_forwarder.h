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

#ifndef EVENT_FORWARDER_H
#define EVENT_FORWARDER_H

#include <klepsydra/core/publisher.h>

namespace kpsr {
template<class T>

/*!
 * @brief The EventForwarder class
 *
 * @copyright 2023 Klepsydra Technologies AG
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-application
 *
 * @details This class provide a facility method to forward an event received on a subscriber. This is mainly used when data needs to be
 * used in a memory middleware and also published to some middleware (e.g., images, state machine publicstates, etc.)
*/
class EventForwarder
{
public:
    /*!
     * @brief EventForwarder
     * @param publisher
     */
    EventForwarder(Publisher<T> *publisher) { _publisher = publisher; }

    /*!
     * @brief onMessageReceived
     * @param event
     */
    void onMessageReceived(const T &event) { _publisher->publish(event); }

private:
    Publisher<T> *_publisher;
};
} // namespace kpsr
#endif
