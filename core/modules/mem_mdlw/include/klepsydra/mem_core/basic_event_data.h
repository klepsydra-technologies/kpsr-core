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

#ifndef BASIC_EVENT_DATA_H
#define BASIC_EVENT_DATA_H

#include <memory>

namespace kpsr {
namespace mem {
template <class T>
/**
 * @brief The EventData struct
 *
 * @copyright Klepsydra Technologies 2019-2020.
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-mem-composition
 *
 * @details wrapper struct to be used when storing events in the queue.
 */
struct EventData {
    /**
     * @brief eventData actual event
     */
    std::shared_ptr<T> eventData;
    /**
     * @brief enqueuedTimeInNs timestamp at which the event was placed in the queue.
     */
    long long unsigned int enqueuedTimeInNs;
};
}
}

#endif // BASIC_EVENT_DATA_H
