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

#ifndef PUBLICATION_STATS_H
#define PUBLICATION_STATS_H

#include <klepsydra/core/function_stats.h>

namespace kpsr {
/*!
 * \brief PublicationStats class.
 *
 * @copyright Klepsydra Technologies 2019-2020.
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-monitoring
 *
 * @details Statistics associated to the performance of the publisher. The messures include the FunctionStats
 * messures plus discarded event and total event object allocations.
 */
struct PublicationStats : public FunctionStats {

    /*!
     * @brief ServiceStats
     * @param name service name to gather stats for.
     * @param type service name to gather stats for.
     */
    PublicationStats(const std::string name, const std::string type)
        : FunctionStats(name)
        , _type(type)
        , _totalEventAllocations(0)
        , _totalDiscardedEvents(0)
    {}

    /*!
     * \brief _type
     */
    const std::string _type;

    /*!
     * \brief _totalEventAllocations
     */
    std::atomic_ullong _totalEventAllocations;

    /*!
     * \brief _totalDiscardedEvents
     */
    std::atomic_ullong _totalDiscardedEvents;

};
}

#endif // PUBLICATION_STATS_H
