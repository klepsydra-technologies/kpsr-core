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
