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
    std::shared_ptr<T> eventData = nullptr;
    /**
     * @brief enqueuedTimeInNs timestamp at which the event was placed in the queue.
     */
    long long unsigned int enqueuedTimeInNs = 0;
};
}
}

#endif // BASIC_EVENT_DATA_H
