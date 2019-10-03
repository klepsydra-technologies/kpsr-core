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

#ifndef DATA_MULTIPLEXER_EVENT_DATA_H
#define DATA_MULTIPLEXER_EVENT_DATA_H

namespace kpsr {
namespace high_performance {
template <class T>
/**
 * @brief The EventData struct
 *
 * @copyright Klepsydra Technologies 2019-2020.
 *
 * @version 2.0.1
 *
 * @ingroup kpsr-high_performance-internal
 *
 * @details Internal wrapper class that is actually stored in the high_performance and eventloop.
 */
struct EventData {

    EventData() {}

    EventData(T eventData)
        : eventData(eventData)
    {}

    /**
     * @brief eventData
     */
    T eventData;

    /**
     * @brief enqueuedTimeInNs
     */
    long long unsigned int enqueuedTimeInNs;
};
}
}

#endif // DATA_MULTIPLEXER_EVENT_DATA_H
