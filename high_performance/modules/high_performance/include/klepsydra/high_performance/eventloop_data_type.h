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

#ifndef EVENTLOOP_DATA_WRAPPER_H
#define EVENTLOOP_DATA_WRAPPER_H

#include <string>
#include <memory>
#include <functional>

namespace kpsr {
namespace high_performance {
/**
 * @brief The EventloopDataWrapper struct
 *
 * @copyright Klepsydra Technologies 2019-2020.
 *
 * @version 2.0.1
 *
 * @ingroup kpsr-eventloop-internal
 *
 */
struct EventloopDataWrapper {
    std::string eventName;
    std::shared_ptr<const void> eventData;
    std::function<void(std::shared_ptr<const void> &)> * releaseFunction;
    long long unsigned int enqueuedTimeInNs;
};
}
}

#endif // EVENTLOOP_DATA_WRAPPER_H
