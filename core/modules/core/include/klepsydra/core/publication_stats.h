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
    PublicationStats(const std::string & name, const std::string & type)
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
