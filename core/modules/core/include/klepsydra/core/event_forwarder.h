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

#ifndef EVENT_FORWARDER_H
#define EVENT_FORWARDER_H

#include <klepsydra/core/publisher.h>

namespace kpsr
{
template <class T>

/*!
 * @brief The EventForwarder class
 *
 * @copyright Klepsydra Technologies 2019-2020.
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
    EventForwarder(Publisher<T> * publisher) {
        _publisher = publisher;
    }

    /*!
     * @brief onMessageReceived
     * @param event
     */
    void onMessageReceived(const T& event) {
        _publisher->publish(event);
    }

private:
    Publisher<T> * _publisher;
};
}
#endif
